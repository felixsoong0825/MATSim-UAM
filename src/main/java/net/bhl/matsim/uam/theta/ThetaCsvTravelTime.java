package net.bhl.matsim.uam.theta;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.util.HashMap;
import java.util.Map;

import org.matsim.api.core.v01.network.Link;
import org.matsim.api.core.v01.population.Person;
import org.matsim.core.router.util.TravelTime;
import org.matsim.core.utils.io.IOUtils;
import org.matsim.vehicles.Vehicle;

/** Piecewise-constant TravelTime backed by an exact theta CSV export. */
public final class ThetaCsvTravelTime implements TravelTime {
	private final Map<String, double[]> values;
	private final double binSize;
	private final int numberOfBins;

	private ThetaCsvTravelTime(Map<String, double[]> values, double binSize, int numberOfBins) {
		this.values = values;
		this.binSize = binSize;
		this.numberOfBins = numberOfBins;
	}

	public static ThetaCsvTravelTime read(String path) {
		try (BufferedReader reader = IOUtils.getBufferedReader(path)) {
			String headerLine = reader.readLine();
			if (headerLine == null) throw new IllegalArgumentException("Empty theta CSV: " + path);
			String[] header = headerLine.split(",", -1);
			if (header.length < 2 || !header[0].equals("link_id"))
				throw new IllegalArgumentException("Invalid theta CSV header: " + path);
			double first = Double.parseDouble(header[1]);
			double binSize = header.length > 2 ? Double.parseDouble(header[2]) - first : 1.0;
			if (first != 0.0 || binSize <= 0.0)
				throw new IllegalArgumentException("Theta bins must start at zero and increase: " + path);

			Map<String, double[]> values = new HashMap<>();
			String line;
			while ((line = reader.readLine()) != null) {
				if (line.isBlank()) continue;
				String[] fields = line.split(",", -1);
				if (fields.length != header.length)
					throw new IllegalArgumentException("Wrong theta column count in " + path);
				double[] row = new double[header.length - 1];
				for (int i = 1; i < fields.length; i++) row[i - 1] = Double.parseDouble(fields[i]);
				if (values.put(fields[0], row) != null)
					throw new IllegalArgumentException("Duplicate theta link " + fields[0]);
			}
			return new ThetaCsvTravelTime(values, binSize, header.length - 1);
		} catch (IOException e) {
			throw new UncheckedIOException("Could not read theta CSV " + path, e);
		}
	}

	@Override
	public double getLinkTravelTime(Link link, double time, Person person, Vehicle vehicle) {
		double[] row = values.get(link.getId().toString());
		if (row == null)
			throw new IllegalArgumentException("Theta has no row for road link " + link.getId());
		int bin = (int) Math.floor(Math.max(0.0, time) / binSize);
		return row[Math.min(bin, numberOfBins - 1)];
	}
}
