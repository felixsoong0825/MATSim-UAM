package net.bhl.matsim.uam.theta;

import static org.junit.Assert.assertEquals;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Set;

import org.junit.Test;
import org.matsim.api.core.v01.Id;
import org.matsim.api.core.v01.Coord;
import org.matsim.api.core.v01.TransportMode;
import org.matsim.api.core.v01.network.Link;
import org.matsim.api.core.v01.network.Network;
import org.matsim.api.core.v01.network.Node;
import org.matsim.core.network.NetworkUtils;
import org.matsim.core.router.util.TravelTime;

public class ThetaTravelTimeWriterTest {
	@Test
	public void writesOnlyModeLinksAndEveryConfiguredBin() throws Exception {
		Network network = NetworkUtils.createNetwork();
		Node n1 = network.getFactory().createNode(Id.createNodeId("n1"), new Coord(0, 0));
		Node n2 = network.getFactory().createNode(Id.createNodeId("n2"), new Coord(1, 0));
		Node n3 = network.getFactory().createNode(Id.createNodeId("n3"), new Coord(2, 0));
		network.addNode(n1);
		network.addNode(n2);
		network.addNode(n3);
		Link car = network.getFactory().createLink(Id.createLinkId("car-link"), n1, n2);
		car.setAllowedModes(Set.of(TransportMode.car));
		Link walk = network.getFactory().createLink(Id.createLinkId("walk-link"), n2, n3);
		walk.setAllowedModes(Set.of(TransportMode.walk));
		Link connector = network.getFactory().createLink(Id.createLinkId("uam_ground-connector"), n2, n3);
		connector.setAllowedModes(Set.of(TransportMode.car));
		network.addLink(car);
		network.addLink(walk);
		network.addLink(connector);

		TravelTime travelTime = (link, time, person, vehicle) -> 10.0 + time / 100.0;
		Path output = Files.createTempFile("theta-", ".csv");
		ThetaTravelTimeWriter.write(output.toString(), network, travelTime, TransportMode.car, 900.0, 1800.0);

		List<String> lines = Files.readAllLines(output);
		assertEquals(List.of(
				"link_id,0.0,900.0,1800.0",
				"car-link,10.0,19.0,28.0"), lines);
	}
}
