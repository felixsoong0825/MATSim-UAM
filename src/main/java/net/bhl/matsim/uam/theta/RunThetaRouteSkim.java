package net.bhl.matsim.uam.theta;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.matsim.api.core.v01.Coord;
import org.matsim.api.core.v01.TransportMode;
import org.matsim.api.core.v01.network.Network;
import org.matsim.api.core.v01.network.Node;
import org.matsim.core.network.NetworkUtils;
import org.matsim.core.network.algorithms.TransportModeNetworkFilter;
import org.matsim.core.router.costcalculators.OnlyTimeDependentTravelDisutility;
import org.matsim.core.router.speedy.SpeedyDijkstraFactory;
import org.matsim.core.router.util.LeastCostPathCalculator;
import org.matsim.core.router.util.LeastCostPathCalculator.Path;
import org.matsim.core.utils.io.IOUtils;

/**
 * Routes every Corsica MILP OD/station-pair candidate with MATSim's native
 * time-dependent shortest-path implementation over one or more supplied
 * damped thetas. Python never reconstructs the graph or runs Dijkstra.
 *
 * Batched manifest mode (2026-09-15): the outer MILP loop's theta-window
 * protocol evaluates a whole POOL of theta snapshots per round (one per
 * design x iteration simulated last round, e.g. 5 designs x 6 iterations =
 * 30 tables) and needs each one routed SEPARATELY (never averaged before
 * routing). Reading network.xml/candidate_sites/zones/demand/neighbors and
 * building the car-only road graph is the expensive, theta-INDEPENDENT part
 * of this program (confirmed: ~40-55s per invocation, almost entirely
 * network parsing/filtering, against a real Corsica network); routing one
 * theta's OD/station pairs against an already-built graph is comparatively
 * cheap. The old one-theta-per-JVM CLI (network+theta+...+output, 9 args)
 * paid that fixed cost once per table -- 30 fresh JVMs, 30 redundant network
 * loads, for 30 routings that could share one. This version loads the
 * network/sites/zones/demand/neighbors ONCE and takes a manifest of
 * theta_path,output_path pairs to route in a loop within that single JVM.
 */
public final class RunThetaRouteSkim {
	private static final double CRUISE_SPEED_M_S = 55.6;
	private static final double VERTICAL_SPEED_M_S = 5.0;

	private record Site(String id, double x, double y, double vtolZ, double groundSpeed,
			double preflight, double postflight, double defaultWaitTime, Node roadNode) {}
	private record Demand(String key, String origin, String destination, int hour, double dMax) {}

	private RunThetaRouteSkim() {}

	public static void main(String[] args) {
		if (args.length != 8) {
			throw new IllegalArgumentException("Usage: RunThetaRouteSkim <network.xml[.gz]> <manifest.csv> "
					+ "<candidate_sites.csv> <zones.csv> <demand.csv> <neighbors.csv> <walkBeelineFactor> "
					+ "<walkSpeedMps>\nmanifest.csv: one \"theta_path,output_path\" pair per line, no header -- "
					+ "the network/sites/zones/demand/neighbors are loaded once and reused for every pair.");
		}
		Network full = NetworkUtils.readNetwork(args[0]);
		Network road = NetworkUtils.createNetwork();
		new TransportModeNetworkFilter(full).filter(road, Set.of(TransportMode.car));
		if (road.getLinks().keySet().stream().anyMatch(id -> id.toString().startsWith("uam_")))
			throw new IllegalArgumentException("Route skims require the base road network, not a generated UAM network");

		Map<String, Coord> zones = readZones(args[3]);
		Map<String, List<String>> neighbors = readNeighbors(args[5]);
		Map<String, Site> sites = readSites(args[2], road);
		List<Demand> demands = readDemand(args[4]);
		double walkFactor = Double.parseDouble(args[6]);
		double walkSpeed = Double.parseDouble(args[7]);

		for (String[] pair : readManifest(args[1])) {
			ThetaCsvTravelTime theta = ThetaCsvTravelTime.read(pair[0]);
			LeastCostPathCalculator router = new SpeedyDijkstraFactory().createPathCalculator(
					road, new OnlyTimeDependentTravelDisutility(theta), theta);
			write(pair[1], demands, zones, neighbors, sites, road, router, walkFactor, walkSpeed);
		}
	}

	private static List<String[]> readManifest(String path) {
		List<String[]> out = new ArrayList<>();
		try (BufferedReader reader = IOUtils.getBufferedReader(path)) {
			String line;
			while ((line = reader.readLine()) != null) {
				if (line.isBlank()) continue;
				String[] parts = line.split(",", -1);
				if (parts.length != 2)
					throw new IllegalArgumentException("Invalid manifest row in " + path + ": " + line);
				out.add(parts);
			}
		} catch (IOException e) {
			throw new UncheckedIOException("Could not read manifest " + path, e);
		}
		return out;
	}

	private static void write(String output, List<Demand> demands, Map<String, Coord> zones,
			Map<String, List<String>> neighbors, Map<String, Site> sites, Network road,
			LeastCostPathCalculator router, double walkFactor, double walkSpeed) {
		try (BufferedWriter writer = IOUtils.getBufferedWriter(output)) {
			writer.write("trip_key,origin,destination,hour,d_max,direct_car_s,station_origin,station_destination,"
					+ "access_mode,access_s,flight_s,egress_mode,egress_s,total_uam_s");
			writer.newLine();
			for (Demand demand : demands) {
				Coord fromCoord = required(zones, demand.origin, "zone");
				Coord toCoord = required(zones, demand.destination, "zone");
				Node fromNode = NetworkUtils.getNearestNode(road, fromCoord);
				Node toNode = NetworkUtils.getNearestNode(road, toCoord);
				double departure = demand.hour * 3600.0;
				double directCar = routeSeconds(router, fromNode, toNode, departure);

				List<String> origins = neighbors.getOrDefault(demand.origin, List.of());
				List<String> destinations = neighbors.getOrDefault(demand.destination, List.of());
				boolean hasPair = origins.stream().anyMatch(j -> destinations.stream().anyMatch(jp -> !j.equals(jp)));
				if (!hasPair) {
					writer.write(csv(demand.key, demand.origin, demand.destination,
							Integer.toString(demand.hour), Double.toString(demand.dMax), Double.toString(directCar),
							"", "", "", "", "", "", "", ""));
					writer.newLine();
					continue;
				}
				Map<String, ModeTime> access = new HashMap<>();
				for (String id : origins) {
					Site site = required(sites, id, "site");
					double connector = distance(site.x, site.y, site.roadNode.getCoord()) / site.groundSpeed;
					double car = routeSeconds(router, fromNode, site.roadNode, departure) + connector;
					double walk = distance(fromCoord, new Coord(site.x, site.y)) * walkFactor / walkSpeed;
					access.put(id, car <= walk ? new ModeTime("car", car) : new ModeTime("walk", walk));
				}

				for (String j : origins) for (String jp : destinations) {
					if (j.equals(jp)) continue;
					Site a = required(sites, j, "site");
					Site b = required(sites, jp, "site");
					ModeTime accessLeg = access.get(j);
					double flight = distance(a.x, a.y, b.x, b.y) / CRUISE_SPEED_M_S
							+ (a.vtolZ + b.vtolZ) / VERTICAL_SPEED_M_S;
					// Origin station wait (2026-09-15): the real router
					// (UAMCachedIntermodalRoutingModule) adds a wait-for-an-
					// available-vehicle term between preflight and the
					// flight leg -- live waiting-time data comes from that
					// simulation's own dispatcher and isn't available to
					// this standalone prediction tool, so this uses each
					// station's own defaultWaitTime, exactly the fallback
					// the real router itself uses whenever live data isn't
					// available (see its catch blocks). Previously omitted
					// entirely, which both understated total UAM duration
					// and queried the wrong theta time-bin for the egress
					// car leg (egressDeparture too early by this amount).
					double egressDeparture = departure + accessLeg.seconds + a.preflight + a.defaultWaitTime + flight;
					double connector = distance(b.x, b.y, b.roadNode.getCoord()) / b.groundSpeed;
					double car = connector + routeSeconds(router, b.roadNode, toNode, egressDeparture + connector);
					double walk = distance(new Coord(b.x, b.y), toCoord) * walkFactor / walkSpeed;
					ModeTime egress = car <= walk ? new ModeTime("car", car) : new ModeTime("walk", walk);
					double total = accessLeg.seconds + a.preflight + a.defaultWaitTime + flight + b.postflight
							+ egress.seconds;
					writer.write(csv(demand.key, demand.origin, demand.destination,
							Integer.toString(demand.hour), Double.toString(demand.dMax), Double.toString(directCar),
							j, jp, accessLeg.mode, Double.toString(accessLeg.seconds), Double.toString(flight),
							egress.mode, Double.toString(egress.seconds), Double.toString(total)));
					writer.newLine();
				}
			}
		} catch (IOException e) {
			throw new UncheckedIOException("Could not write route skim " + output, e);
		}
	}

	private record ModeTime(String mode, double seconds) {}

	private static double routeSeconds(LeastCostPathCalculator router, Node from, Node to, double departure) {
		if (from.equals(to)) return 0.0;
		Path path = router.calcLeastCostPath(from, to, departure, null, null);
		if (path == null) return Double.POSITIVE_INFINITY;
		return path.travelTime;
	}

	private static Map<String, Coord> readZones(String path) {
		Map<String, Coord> out = new HashMap<>();
		for (Map<String, String> row : rows(path)) out.put(row.get("zone_id"),
				new Coord(number(row, "x"), number(row, "y")));
		return out;
	}

	private static Map<String, List<String>> readNeighbors(String path) {
		Map<String, List<String>> out = new HashMap<>();
		for (Map<String, String> row : rows(path))
			out.computeIfAbsent(row.get("zone_id"), ignored -> new ArrayList<>()).add(row.get("station_id"));
		return out;
	}

	private static List<Demand> readDemand(String path) {
		List<Demand> out = new ArrayList<>();
		for (Map<String, String> row : rows(path)) out.add(new Demand(row.get("trip_key"),
				row.get("start_facility_id"), row.get("end_facility_id"),
				Integer.parseInt(row.get("hour")), number(row, "d_max")));
		return out;
	}

	private static Map<String, Site> readSites(String path, Network road) {
		Map<String, Site> out = new HashMap<>();
		for (Map<String, String> row : rows(path)) {
			double x = number(row, "x"), y = number(row, "y");
			String id = row.get("station_id");
			out.put(id, new Site(id, x, y, number(row, "vtol_z"), number(row, "ground_access_freespeed"),
					number(row, "preflighttime"), number(row, "postflighttime"), number(row, "defaultwaittime"),
					NetworkUtils.getNearestNode(road, new Coord(x, y))));
		}
		return out;
	}

	private static List<Map<String, String>> rows(String path) {
		try (BufferedReader reader = IOUtils.getBufferedReader(path)) {
			String first = reader.readLine();
			if (first == null) throw new IllegalArgumentException("Empty CSV " + path);
			String[] header = first.replace("\ufeff", "").split(",", -1);
			List<Map<String, String>> out = new ArrayList<>();
			String line;
			while ((line = reader.readLine()) != null) {
				if (line.isBlank()) continue;
				String[] fields = line.split(",", -1);
				if (fields.length != header.length) throw new IllegalArgumentException("Invalid CSV row in " + path);
				Map<String, String> row = new LinkedHashMap<>();
				for (int i = 0; i < header.length; i++) row.put(header[i], fields[i]);
				out.add(row);
			}
			return out;
		} catch (IOException e) {
			throw new UncheckedIOException("Could not read " + path, e);
		}
	}

	private static double number(Map<String, String> row, String field) {
		return Double.parseDouble(row.get(field));
	}

	private static double distance(double x, double y, Coord b) {
		return Math.hypot(b.getX() - x, b.getY() - y);
	}

	private static double distance(double ax, double ay, double bx, double by) {
		return Math.hypot(bx - ax, by - ay);
	}

	private static double distance(Coord a, Coord b) {
		return Math.hypot(b.getX() - a.getX(), b.getY() - a.getY());
	}

	private static <T> T required(Map<String, T> map, String key, String kind) {
		T value = map.get(key);
		if (value == null) throw new IllegalArgumentException("Unknown " + kind + " " + key);
		return value;
	}

	private static String csv(String... fields) {
		StringBuilder line = new StringBuilder();
		for (int i = 0; i < fields.length; i++) {
			if (i > 0) line.append(',');
			line.append(quote(fields[i]));
		}
		return line.toString();
	}

	// RFC4180 minimal quoting (2026-09-15): every field written here is
	// currently a station/zone/trip id or a plain number, none of which
	// contain a comma or quote today, but writing plain String.join(",",
	// fields) meant this format was never actually SAFE against one --
	// Python's own reader (java_route_skims.py's csv.DictReader) already
	// assumes proper CSV quoting, so this closes the gap on the writer side
	// to match rather than relying on the input data never containing a
	// comma.
	private static String quote(String field) {
		if (field.indexOf(',') < 0 && field.indexOf('"') < 0 && field.indexOf('\n') < 0) return field;
		return '"' + field.replace("\"", "\"\"") + '"';
	}
}
