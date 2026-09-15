package net.bhl.matsim.uam.theta;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.matsim.api.core.v01.network.Network;
import org.matsim.core.config.Config;
import org.matsim.core.config.groups.TravelTimeCalculatorConfigGroup;
import org.matsim.core.controler.events.IterationEndsEvent;
import org.matsim.core.controler.listener.IterationEndsListener;

import com.google.inject.Inject;

/** Exports the exact in-memory car TravelTime queried by MATSim routing. */
public final class ThetaExportListener implements IterationEndsListener {
	private static final Logger log = LogManager.getLogger(ThetaExportListener.class);

	private final Network network;
	private final ThetaExportConfigGroup exportConfig;
	private final TravelTimeCalculatorConfigGroup travelTimeConfig;

	@Inject
	public ThetaExportListener(Network network, Config config) {
		this.network = network;
		this.exportConfig = ThetaExportConfigGroup.get(config);
		this.travelTimeConfig = config.travelTimeCalculator();
	}

	@Override
	public void notifyIterationEnds(IterationEndsEvent event) {
		int iteration = event.getIteration();
		if (!exportConfig.exportsIteration(iteration)) {
			return;
		}

		String path = event.getServices().getControlerIO()
				.getIterationFilename(iteration, exportConfig.getFileName());
		ThetaTravelTimeWriter.write(path, network, event.getServices().getLinkTravelTimes(),
				exportConfig.getMode(), travelTimeConfig.getTraveltimeBinSize(), travelTimeConfig.getMaxTime());
		log.info("Exported theta for iteration {} to {}", iteration, path);
	}
}
