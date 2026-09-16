package net.bhl.matsim.uam.theta;

import java.io.BufferedInputStream;
import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.ByteBuffer;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.matsim.api.core.v01.network.Link;
import org.matsim.api.core.v01.network.Network;
import org.matsim.api.core.v01.population.Person;
import org.matsim.core.router.util.TravelTime;
import org.matsim.core.utils.io.IOUtils;
import org.matsim.vehicles.Vehicle;

/**
 * Piecewise-constant TravelTime backed by an exact theta CSV export.
 *
 * Indexed by MATSim's dense link index rather than by link-id String
 * (2026-09-16). getLinkTravelTime is the innermost frame of every Dijkstra
 * expansion -- a B=16 Corsica round routes 105,000-200,000 shortest paths per
 * theta state, so this method runs tens of millions of times per state, and a
 * HashMap&lt;String, double[]&gt; lookup paid a String hash plus an equals on
 * every single one. An array subscript removes that from the hot path
 * entirely.
 */
public final class ThetaCsvTravelTime implements TravelTime {
	private final double[][] byLinkIndex;
	private final double binSize;
	private final int numberOfBins;

	/**
	 * Link id -&gt; dense index, built once from the road network and shared by
	 * every theta table.
	 *
	 * Resolving ids through this instead of {@code Id.create} matters for
	 * correctness, not just speed: RunThetaRouteSkim parses the theta window's
	 * tables in parallel, and {@code Id.create} would mutate MATSim's global id
	 * registry from several threads at once. Every id here already exists --
	 * the network was read first -- so this is a pure read.
	 */
	public static final class LinkIndex {
		private final Map<String, Integer> indexById;
		private final int size;

		private LinkIndex(Map<String, Integer> indexById, int size) {
			this.indexById = indexById;
			this.size = size;
		}

		public static LinkIndex of(Network road) {
			Map<String, Integer> indexById = new HashMap<>(road.getLinks().size() * 2);
			int max = -1;
			for (Link link : road.getLinks().values()) {
				int index = link.getId().index();
				indexById.put(link.getId().toString(), index);
				if (index > max) max = index;
			}
			return new LinkIndex(indexById, max + 1);
		}

		/** -1 when the id is not part of the routing network. */
		int indexOf(String linkId) {
			Integer index = indexById.get(linkId);
			return index == null ? -1 : index;
		}

		int size() {
			return size;
		}
	}

	private ThetaCsvTravelTime(double[][] byLinkIndex, double binSize, int numberOfBins) {
		this.byLinkIndex = byLinkIndex;
		this.binSize = binSize;
		this.numberOfBins = numberOfBins;
	}

	public static ThetaCsvTravelTime read(String path, LinkIndex linkIndex) {
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

			double[][] byLinkIndex = new double[linkIndex.size()][];
			// Rows for links outside the routing network are kept aside rather
			// than dropped, so a duplicate anywhere in the file is still an
			// error -- the previous String-keyed map caught those too.
			List<String> extra = new ArrayList<>();
			String line;
			while ((line = reader.readLine()) != null) {
				if (line.isBlank()) continue;
				String[] fields = line.split(",", -1);
				if (fields.length != header.length)
					throw new IllegalArgumentException("Wrong theta column count in " + path);
				double[] row = new double[header.length - 1];
				for (int i = 1; i < fields.length; i++) row[i - 1] = Double.parseDouble(fields[i]);
				int index = linkIndex.indexOf(fields[0]);
				if (index < 0) {
					extra.add(fields[0]);
					continue;
				}
				if (byLinkIndex[index] != null)
					throw new IllegalArgumentException("Duplicate theta link " + fields[0]);
				byLinkIndex[index] = row;
			}
			if (extra.size() != new java.util.HashSet<>(extra).size())
				throw new IllegalArgumentException("Duplicate theta link outside the routing network in " + path);
			return new ThetaCsvTravelTime(byLinkIndex, binSize, header.length - 1);
		} catch (IOException e) {
			throw new UncheckedIOException("Could not read theta CSV " + path, e);
		}
	}

	private static final byte[] BINARY_MAGIC = "UAMTHETA".getBytes(StandardCharsets.US_ASCII);
	private static final int BINARY_VERSION = 1;

	/** Dispatches on the file's extension: ".bin" is the binary handoff format. */
	public static ThetaCsvTravelTime read(String path, LinkIndex linkIndex, boolean binary) {
		return binary ? readBinary(path, linkIndex) : read(path, linkIndex);
	}

	/**
	 * Read the binary handoff format written by theta_table.write_theta_binary.
	 *
	 * The CSV path costs the Python writer 3.25 s per table in decimal
	 * formatting alone (8.5 million float64s), which Java then parses straight
	 * back. Handing over the raw big-endian block instead removes both sides of
	 * that round trip. See write_theta_binary for the layout.
	 */
	public static ThetaCsvTravelTime readBinary(String path, LinkIndex linkIndex) {
		try (DataInputStream in = new DataInputStream(
				new BufferedInputStream(new FileInputStream(path), 1 << 20))) {
			byte[] magic = in.readNBytes(BINARY_MAGIC.length);
			if (!java.util.Arrays.equals(magic, BINARY_MAGIC))
				throw new IllegalArgumentException("Not a theta binary file: " + path);
			int version = in.readInt();
			if (version != BINARY_VERSION)
				throw new IllegalArgumentException(
						"Unsupported theta binary version " + version + ": " + path);
			int numLinks = in.readInt();
			int numBins = in.readInt();
			if (numLinks < 0 || numBins <= 0)
				throw new IllegalArgumentException("Invalid theta binary dimensions: " + path);
			double binSize = in.readDouble();
			if (binSize <= 0.0)
				throw new IllegalArgumentException("Theta bins must increase: " + path);

			String[] ids = new String[numLinks];
			for (int i = 0; i < numLinks; i++) {
				int length = in.readInt();
				if (length < 0) throw new IllegalArgumentException("Invalid link id length: " + path);
				ids[i] = new String(in.readNBytes(length), StandardCharsets.UTF_8);
			}

			double[][] byLinkIndex = new double[linkIndex.size()][];
			byte[] block = new byte[numBins * Double.BYTES];
			for (int i = 0; i < numLinks; i++) {
				in.readFully(block);
				int index = linkIndex.indexOf(ids[i]);
				if (index < 0) continue;   // link is not part of the routing network
				if (byLinkIndex[index] != null)
					throw new IllegalArgumentException("Duplicate theta link " + ids[i]);
				double[] row = new double[numBins];
				ByteBuffer.wrap(block).asDoubleBuffer().get(row);
				byLinkIndex[index] = row;
			}
			return new ThetaCsvTravelTime(byLinkIndex, binSize, numBins);
		} catch (IOException e) {
			throw new UncheckedIOException("Could not read theta binary " + path, e);
		}
	}

	@Override
	public double getLinkTravelTime(Link link, double time, Person person, Vehicle vehicle) {
		int index = link.getId().index();
		double[] row = index >= 0 && index < byLinkIndex.length ? byLinkIndex[index] : null;
		if (row == null)
			throw new IllegalArgumentException("Theta has no row for road link " + link.getId());
		int bin = (int) Math.floor(Math.max(0.0, time) / binSize);
		return row[Math.min(bin, numberOfBins - 1)];
	}
}
