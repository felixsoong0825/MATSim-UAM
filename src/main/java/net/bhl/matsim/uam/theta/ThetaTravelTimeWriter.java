package net.bhl.matsim.uam.theta;

import java.io.BufferedWriter;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.util.Comparator;

import org.matsim.api.core.v01.network.Link;
import org.matsim.api.core.v01.network.Network;
import org.matsim.core.router.util.TravelTime;
import org.matsim.core.utils.io.IOUtils;

/** Writes a compact, wide link x time-bin table. */
public final class ThetaTravelTimeWriter {
	private ThetaTravelTimeWriter() {
	}

	public static int numberOfBins(double maxTime, double binSize) {
		return (int) Math.ceil(maxTime / binSize) + 1;
	}

	public static void write(String path, Network network, TravelTime travelTime, String mode,
			double binSize, double maxTime) {
		int numberOfBins = numberOfBins(maxTime, binSize);
		try (BufferedWriter writer = IOUtils.getBufferedWriter(path)) {
			writer.write("link_id");
			for (int bin = 0; bin < numberOfBins; bin++) {
				writer.write(',');
				writer.write(Double.toString(bin * binSize));
			}
			writer.newLine();

			network.getLinks().values().stream()
					.filter(link -> link.getAllowedModes().contains(mode))
					// UAM scenario generation adds design-specific, dangling station
					// connectors to the car network.  They are not part of the common
					// road-state theta and would make theta tables from two designs have
					// different row sets, so export only the stable base-road links.
					.filter(link -> !link.getId().toString().startsWith("uam_"))
					.sorted(Comparator.comparing(link -> link.getId().toString()))
					.forEach(link -> writeLink(writer, link, travelTime, binSize, numberOfBins));
		} catch (IOException e) {
			throw new UncheckedIOException("Could not write theta table to " + path, e);
		}
	}

	private static void writeLink(BufferedWriter writer, Link link, TravelTime travelTime,
			double binSize, int numberOfBins) {
		try {
			writer.write(escapeCsv(link.getId().toString()));
			for (int bin = 0; bin < numberOfBins; bin++) {
				writer.write(',');
				writer.write(Double.toString(
						travelTime.getLinkTravelTime(link, bin * binSize, null, null)));
			}
			writer.newLine();
		} catch (IOException e) {
			throw new UncheckedIOException(e);
		}
	}

	private static String escapeCsv(String value) {
		if (value.indexOf(',') < 0 && value.indexOf('"') < 0 && value.indexOf('\n') < 0
				&& value.indexOf('\r') < 0) {
			return value;
		}
		return '"' + value.replace("\"", "\"\"") + '"';
	}
}
