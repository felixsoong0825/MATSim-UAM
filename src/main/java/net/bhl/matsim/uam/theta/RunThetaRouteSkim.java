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
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ForkJoinPool;

import org.matsim.api.core.v01.Coord;
import org.matsim.api.core.v01.Id;
import org.matsim.api.core.v01.Scenario;
import org.matsim.api.core.v01.TransportMode;
import org.matsim.api.core.v01.network.Link;
import org.matsim.api.core.v01.network.Network;
import org.matsim.api.core.v01.network.Node;
import org.matsim.api.core.v01.population.Leg;
import org.matsim.api.core.v01.population.PlanElement;
import org.matsim.core.config.Config;
import org.matsim.core.config.ConfigUtils;
import org.matsim.core.network.NetworkUtils;
import org.matsim.core.network.algorithms.TransportModeNetworkFilter;
import org.matsim.core.router.DefaultRoutingRequest;
import org.matsim.core.router.RoutingModule;
import org.matsim.core.router.TeleportationRoutingModule;
import org.matsim.core.router.costcalculators.OnlyTimeDependentTravelDisutility;
import org.matsim.core.router.speedy.SpeedyDijkstraFactory;
import org.matsim.core.router.util.LeastCostPathCalculator;
import org.matsim.core.router.util.LeastCostPathCalculator.Path;
import org.matsim.core.scenario.ScenarioUtils;
import org.matsim.core.utils.io.IOUtils;
import org.matsim.core.utils.timing.TimeInterpretation;
import org.matsim.core.utils.timing.TimeTracker;
import org.matsim.facilities.FacilitiesUtils;
import org.matsim.pt.routes.TransitPassengerRoute;
import org.matsim.utils.objectattributes.attributable.AttributesImpl;

import ch.sbb.matsim.config.SwissRailRaptorConfigGroup;
import ch.sbb.matsim.routing.pt.raptor.DefaultRaptorInVehicleCostCalculator;
import ch.sbb.matsim.routing.pt.raptor.DefaultRaptorIntermodalAccessEgress;
import ch.sbb.matsim.routing.pt.raptor.DefaultRaptorParametersForPerson;
import ch.sbb.matsim.routing.pt.raptor.DefaultRaptorStopFinder;
import ch.sbb.matsim.routing.pt.raptor.DefaultRaptorTransferCostCalculator;
import ch.sbb.matsim.routing.pt.raptor.LeastCostRaptorRouteSelector;
import ch.sbb.matsim.routing.pt.raptor.RaptorStaticConfig;
import ch.sbb.matsim.routing.pt.raptor.RaptorUtils;
import ch.sbb.matsim.routing.pt.raptor.SwissRailRaptor;
import ch.sbb.matsim.routing.pt.raptor.SwissRailRaptorData;
import ch.sbb.matsim.routing.pt.raptor.SwissRailRaptorRoutingModule;
import net.bhl.matsim.uam.config.UAMConfigGroup;

/**
 * Routes every MILP OD/station-pair candidate with MATSim's native
 * time-dependent shortest-path implementation over one or more supplied
 * damped thetas. Python never reconstructs the graph or runs Dijkstra.
 *
 * Batched manifest mode (2026-09-15): the outer MILP loop's theta-window
 * protocol evaluates a whole POOL of theta snapshots per round (one per
 * design x iteration simulated last round, e.g. 5 designs x 6 iterations =
 * 30 tables) and needs each one routed SEPARATELY (never averaged before
 * routing). The old one-theta-per-JVM CLI paid the setup cost once per
 * table -- 30 fresh JVMs, 30 redundant network loads. This version loads the
 * network/sites/zones/demand/neighbors ONCE and takes a manifest of
 * theta_path,output_path pairs.
 *
 * Correction (2026-09-16): the note that setup is "~40-55s per invocation"
 * and dominates wall time was wrong by an order of magnitude. Measured on the
 * real Corsica network, JVM start through the QuadTree takes 2.2 s, while a
 * single theta state takes ~64 s. Batching saves 30 x 2.2 s, not 30 x 50 s.
 * Routing is what costs: one state is 105,000-200,000 shortest paths (8,997
 * demand rows, 25,553 access legs and 70,309 station pairs, each 1-2
 * Dijkstras).
 *
 * States are therefore routed in PARALLEL. They are independent by
 * construction -- each has its own theta, its own LeastCostPathCalculator and
 * its own output file, and the road network, zones, sites, neighbors, demand
 * and PT times are all read-only once setup is done. Thread count comes from
 * -Dtheta.skim.threads, defaulting to half the available processors (a
 * 28-thread box was previously using 0.98 of one core for 35 minutes).
 */
public final class RunThetaRouteSkim {
	private static final double CRUISE_SPEED_M_S = 55.6;
	private static final double VERTICAL_SPEED_M_S = 5.0;

	private record Site(String id, double x, double y, double vtolZ, double groundSpeed,
			double preflight, double postflight, double defaultWaitTime, Node roadNode) {}
	private record Zone(Coord coord, Link roadLink) {}
	private record Demand(String key, String origin, String destination, int hour, double dMax) {}
	private record PtRouter(RoutingModule module, TimeInterpretation timeInterpretation) {}
	private record PtTime(double inVehicleSeconds, double waitAndWalkSeconds) {}

	private RunThetaRouteSkim() {}

	public static void main(String[] args) {
		if (args.length != 8 && args.length != 9) {
			throw new IllegalArgumentException("Usage: RunThetaRouteSkim <network.xml[.gz]> <manifest.csv> "
					+ "<candidate_sites.csv> <zones.csv> <demand.csv> <neighbors.csv> <walkBeelineFactor> "
					+ "<walkSpeedMps> [ptConfig.xml]\nmanifest.csv: one \"theta_path,output_path\" pair per line, no header -- "
					+ "the network/sites/zones/demand/neighbors are loaded once and reused for every pair.");
		}
		Network full = NetworkUtils.readNetwork(args[0]);
		Network road = NetworkUtils.createNetwork();
		new TransportModeNetworkFilter(full).filter(road, Set.of(TransportMode.car));
		if (road.getLinks().keySet().stream().anyMatch(id -> id.toString().startsWith("uam_")))
			throw new IllegalArgumentException("Route skims require the base road network, not a generated UAM network");

		Map<String, Zone> zones = readZones(args[3], full, road);
		Map<String, List<String>> neighbors = readNeighbors(args[5]);
		Map<String, Site> sites = readSites(args[2], road);
		List<Demand> demands = readDemand(args[4]);
		double walkFactor = Double.parseDouble(args[6]);
		double walkSpeed = Double.parseDouble(args[7]);
		PtRouter ptRouter = args.length == 9 ? createPtRouter(args[8], walkFactor, walkSpeed) : null;
		Map<String, PtTime> ptTimes = routePtTrips(ptRouter, demands, zones);

		List<String[]> manifest = readManifest(args[1]);
		ThetaCsvTravelTime.LinkIndex linkIndex = ThetaCsvTravelTime.LinkIndex.of(road);
		int threads = skimThreads(manifest.size());
		System.out.println("Routing " + manifest.size() + " theta states on " + threads + " thread(s)");
		routeAll(manifest, threads, pair -> {
			ThetaCsvTravelTime theta = ThetaCsvTravelTime.read(
					pair[0], linkIndex, pair[0].endsWith(".bin"));
			LeastCostPathCalculator router = new SpeedyDijkstraFactory().createPathCalculator(
					road, new OnlyTimeDependentTravelDisutility(theta), theta);
			write(pair[1], demands, zones, neighbors, sites, router, ptTimes, walkFactor, walkSpeed);
		});
	}

	/**
	 * Half the available processors unless -Dtheta.skim.threads says otherwise,
	 * never more than there are states to route. Each thread holds its own
	 * theta table (70,129 links x 121 bins x 8 B = 68 MiB on Corsica) plus its
	 * own routing graph, so the JVM needs headroom for that many copies.
	 */
	private static int skimThreads(int states) {
		int configured = Integer.getInteger("theta.skim.threads", 0);
		if (configured > 0) return Math.max(1, Math.min(configured, states));
		int cores = Runtime.getRuntime().availableProcessors();
		return Math.max(1, Math.min(states, Math.max(1, cores / 2)));
	}

	private static void routeAll(List<String[]> manifest, int threads, java.util.function.Consumer<String[]> route) {
		if (threads <= 1) {
			manifest.forEach(route);
			return;
		}
		ForkJoinPool pool = new ForkJoinPool(threads);
		try {
			pool.submit(() -> manifest.parallelStream().forEach(route)).get();
		} catch (InterruptedException e) {
			Thread.currentThread().interrupt();
			throw new IllegalStateException("Theta routing interrupted", e);
		} catch (ExecutionException e) {
			// Surface the routing failure itself, not the pool's wrapper --
			// callers match on the message of the original exception.
			Throwable cause = e.getCause();
			if (cause instanceof RuntimeException runtime) throw runtime;
			if (cause instanceof Error error) throw error;
			throw new IllegalStateException("Theta routing failed", cause);
		} finally {
			pool.shutdown();
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

	private static void write(String output, List<Demand> demands, Map<String, Zone> zones,
			Map<String, List<String>> neighbors, Map<String, Site> sites,
			LeastCostPathCalculator router, Map<String, PtTime> ptTimes,
			double walkFactor, double walkSpeed) {
		try (BufferedWriter writer = IOUtils.getBufferedWriter(output)) {
			writer.write("trip_key,origin,destination,hour,d_max,direct_car_s,pt_in_vehicle_s,pt_wait_walk_s,"
					+ "station_origin,station_destination,"
					+ "access_mode,access_s,flight_s,egress_mode,egress_s,selection_uam_s,total_uam_s");
			writer.newLine();
			for (Demand demand : demands) {
				Zone fromZone = required(zones, demand.origin, "zone");
				Zone toZone = required(zones, demand.destination, "zone");
				Link fromLink = fromZone.roadLink;
				Link toLink = toZone.roadLink;
				double departure = demand.hour * 3600.0;
				double directCar = routeSeconds(router, fromLink.getToNode(), toLink.getFromNode(), departure);
				PtTime pt = ptTimes.get(demand.key);
				String ptInVehicle = pt == null ? "" : Double.toString(pt.inVehicleSeconds);
				String ptWaitWalk = pt == null ? "" : Double.toString(pt.waitAndWalkSeconds);

				List<String> origins = neighbors.getOrDefault(demand.origin, List.of());
				List<String> destinations = neighbors.getOrDefault(demand.destination, List.of());
				boolean hasPair = origins.stream().anyMatch(j -> destinations.stream().anyMatch(jp -> !j.equals(jp)));
				if (!hasPair) {
					writer.write(csv(demand.key, demand.origin, demand.destination,
							Integer.toString(demand.hour), Double.toString(demand.dMax), Double.toString(directCar),
							ptInVehicle, ptWaitWalk,
							"", "", "", "", "", "", "", "", ""));
					writer.newLine();
					continue;
				}
				Map<String, AccessChoice> access = new HashMap<>();
				for (String id : origins) {
					Site site = required(sites, id, "site");
					double connector = distance(site.x, site.y, site.roadNode.getCoord()) / site.groundSpeed;
					double selectionCar = routeSeconds(
							router, fromLink.getFromNode(), site.roadNode, departure) + connector;
					double selectionWalk = distance(fromLink.getCoord(), new Coord(site.x, site.y))
							* walkFactor / walkSpeed;
					if (selectionCar <= selectionWalk) {
						double actualCar = routeSeconds(
								router, fromLink.getToNode(), site.roadNode, departure) + connector;
						access.put(id, new AccessChoice("car", selectionCar, actualCar, actualCar));
					} else {
						double actualClock = distance(fromLink.getCoord(), new Coord(site.x, site.y))
								/ walkSpeed + 1.0;
						// UAMCachedIntermodalRoutingModule advances currentTime by this
						// raw teleport time, then resets the returned access-walk leg to 0.
						access.put(id, new AccessChoice("walk", selectionWalk, 0.0, actualClock));
					}
				}

				for (String j : origins) for (String jp : destinations) {
					if (j.equals(jp)) continue;
					Site a = required(sites, j, "site");
					Site b = required(sites, jp, "site");
					AccessChoice accessLeg = access.get(j);
					double flight = distance(a.x, a.y, b.x, b.y) / CRUISE_SPEED_M_S
							+ (a.vtolZ + b.vtolZ) / VERTICAL_SPEED_M_S;
					double process = a.preflight + b.postflight;
					// This is the clock used by UAMMinTravelTimeStrategy when
					// comparing station pairs. Waiting is not part of that estimate.
					double selectionEgressDeparture = departure + accessLeg.selectionSeconds + flight + process;
					double connector = distance(b.x, b.y, b.roadNode.getCoord()) / b.groundSpeed;
					double selectionCar = connector + routeSeconds(
							router, b.roadNode, toLink.getToNode(), selectionEgressDeparture + connector);
					double selectionWalk = distance(new Coord(b.x, b.y), toLink.getCoord())
							* walkFactor / walkSpeed;
					String egressMode = selectionCar <= selectionWalk ? "car" : "walk";
					double selectionEgress = egressMode.equals("car") ? selectionCar : selectionWalk;

					// The wait advances only the live routing clock. It is not a
					// returned plan element and therefore is not directly scored by
					// IdfStyleTripEstimator's TimeTracker.
					double actualEgressDeparture = departure + accessLeg.actualClockSeconds
							+ a.preflight + a.defaultWaitTime + flight + b.postflight;
					double actualEgress;
					if (egressMode.equals("car")) {
						actualEgress = connector + routeSeconds(
								router, b.roadNode, toLink.getFromNode(), actualEgressDeparture + connector);
					} else {
						actualEgress = distance(new Coord(b.x, b.y), toLink.getCoord()) / walkSpeed + 1.0;
					}
					double selectionTotal = accessLeg.selectionSeconds + flight + process + selectionEgress;
					double total = accessLeg.actualDurationSeconds + a.preflight + flight
							+ b.postflight + actualEgress;
					writer.write(csv(demand.key, demand.origin, demand.destination,
							Integer.toString(demand.hour), Double.toString(demand.dMax), Double.toString(directCar),
							ptInVehicle, ptWaitWalk,
							j, jp, accessLeg.mode, Double.toString(accessLeg.actualDurationSeconds), Double.toString(flight),
							egressMode, Double.toString(actualEgress), Double.toString(selectionTotal),
							Double.toString(total)));
					writer.newLine();
				}
			}
		} catch (IOException e) {
			throw new UncheckedIOException("Could not write route skim " + output, e);
		}
	}

	private static PtRouter createPtRouter(String configPath, double walkFactor, double walkSpeed) {
		Config config = ConfigUtils.loadConfig(configPath,
				new UAMConfigGroup(), new SwissRailRaptorConfigGroup());
		if (!config.transit().isUseTransit()) {
			throw new IllegalArgumentException("PT skim config has useTransit=false: " + configPath);
		}
		Scenario scenario = ScenarioUtils.createScenario(config);
		ScenarioUtils.loadScenario(scenario);

		RaptorStaticConfig staticConfig = RaptorUtils.createStaticConfig(config);
		SwissRailRaptorData data = SwissRailRaptorData.create(
				scenario.getTransitSchedule(), null, staticConfig, scenario.getNetwork(), null);
		RoutingModule transitWalk = new TeleportationRoutingModule(
				TransportMode.walk, scenario, walkSpeed, walkFactor);
		Map<String, RoutingModule> accessRouters = new HashMap<>();
		accessRouters.put(TransportMode.walk, transitWalk);
		SwissRailRaptor raptor = new SwissRailRaptor(data,
				new DefaultRaptorParametersForPerson(config),
				new LeastCostRaptorRouteSelector(),
				new DefaultRaptorStopFinder(new DefaultRaptorIntermodalAccessEgress(), accessRouters),
				new DefaultRaptorInVehicleCostCalculator(),
				new DefaultRaptorTransferCostCalculator());
		RoutingModule module = new SwissRailRaptorRoutingModule(
				raptor, scenario.getTransitSchedule(), scenario.getNetwork(), transitWalk);
		return new PtRouter(module, TimeInterpretation.create(config));
	}

	private static Map<String, PtTime> routePtTrips(
			PtRouter router, List<Demand> demands, Map<String, Zone> zones) {
		if (router == null) return Map.of();
		Map<String, PtTime> result = new HashMap<>();
		for (Demand demand : demands) {
			if (result.containsKey(demand.key)) continue;
			Zone from = required(zones, demand.origin, "zone");
			Zone to = required(zones, demand.destination, "zone");
			List<? extends PlanElement> elements = router.module.calcRoute(DefaultRoutingRequest.of(
					FacilitiesUtils.wrapLinkAndCoord(from.roadLink, from.coord),
					FacilitiesUtils.wrapLinkAndCoord(to.roadLink, to.coord),
					demand.hour * 3600.0, null, new AttributesImpl()));
			if (elements != null) {
				result.put(demand.key, splitPtTime(
						elements, demand.hour * 3600.0, router.timeInterpretation));
			}
		}
		return result;
	}

	/** Exact seconds-level port of IdfStyleTripEstimator's PT time split. */
	private static PtTime splitPtTime(List<? extends PlanElement> elements,
			double departureTime, TimeInterpretation timeInterpretation) {
		double inVehicleSeconds = 0.0;
		double waitAndWalkSeconds = 0.0;
		TimeTracker tracker = new TimeTracker(timeInterpretation);
		tracker.setTime(departureTime);
		for (PlanElement element : elements) {
			if (element instanceof Leg leg) {
				double legStart = tracker.getTime().seconds();
				tracker.addElement(element);
				double legDuration = tracker.getTime().seconds() - legStart;
				if (leg.getRoute() instanceof TransitPassengerRoute ptRoute) {
					double boardingTime = ptRoute.getBoardingTime().orElse(legStart);
					double waitSeconds = Math.max(0.0, boardingTime - legStart);
					waitAndWalkSeconds += waitSeconds;
					inVehicleSeconds += Math.max(0.0, legDuration - waitSeconds);
				} else {
					waitAndWalkSeconds += legDuration;
				}
			} else {
				tracker.addElement(element);
			}
		}
		return new PtTime(inVehicleSeconds, waitAndWalkSeconds);
	}

	private record AccessChoice(String mode, double selectionSeconds,
			double actualDurationSeconds, double actualClockSeconds) {}

	private static double routeSeconds(LeastCostPathCalculator router, Node from, Node to, double departure) {
		if (from.equals(to)) return 0.0;
		Path path = router.calcLeastCostPath(from, to, departure, null, null);
		if (path == null) return Double.POSITIVE_INFINITY;
		return path.travelTime;
	}

	private static Map<String, Zone> readZones(String path, Network full, Network road) {
		Map<String, Zone> out = new HashMap<>();
		for (Map<String, String> row : rows(path)) {
			String linkId = row.get("link_id");
			if (linkId == null || linkId.isBlank())
				throw new IllegalArgumentException(
						"zones CSV must contain exact MATSim link_id values: " + path);
			Id<Link> id = Id.createLinkId(linkId);
			Link sourceLink = full.getLinks().get(id);
			if (sourceLink == null)
				throw new IllegalArgumentException(
						"Unknown network link " + linkId + " for zone " + row.get("zone_id"));
			Link link = road.getLinks().get(id);
			if (link == null) {
				// UAMStrategyUtils applies the same fallback when an activity's
				// exact link is not part of the car-filtered routing network.
				link = NetworkUtils.getNearestLinkExactly(road, sourceLink.getCoord());
			}
			out.put(row.get("zone_id"), new Zone(
					new Coord(number(row, "x"), number(row, "y")), link));
		}
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
