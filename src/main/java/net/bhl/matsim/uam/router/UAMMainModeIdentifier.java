package net.bhl.matsim.uam.router;

import net.bhl.matsim.uam.run.UAMConstants;
import org.matsim.api.core.v01.TransportMode;
import org.matsim.api.core.v01.population.Leg;
import org.matsim.api.core.v01.population.PlanElement;
import org.matsim.core.router.AnalysisMainModeIdentifier;
import org.matsim.core.router.MainModeIdentifier;

import java.util.List;

/**
 * This class identifies if a trip is using UAM or not.
 *
 * Implements AnalysisMainModeIdentifier (which extends MainModeIdentifier) so
 * a single instance can be bound to both interfaces: MainModeIdentifier is
 * used during routing/replanning, AnalysisMainModeIdentifier is the separate
 * binding key ModeStatsControlerListener (iteration mode-share reporting)
 * requires -- without this, it falls back to DefaultAnalysisMainModeIdentifier,
 * which throws "unknown modes" for uam/access_uam_car/egress_uam_car legs.
 *
 * @author balacmi (Milos Balac), RRothfeld (Raoul Rothfeld)
 */
public class UAMMainModeIdentifier implements AnalysisMainModeIdentifier {
	private final MainModeIdentifier defaultModeIdentifier;

	public UAMMainModeIdentifier(final MainModeIdentifier defaultModeIdentifier) {
		this.defaultModeIdentifier = defaultModeIdentifier;
	}

	@Override
	public String identifyMainMode(List<? extends PlanElement> tripElements) {
		for (PlanElement pe : tripElements) {
			if (pe instanceof Leg && (((Leg) pe).getMode().equals(UAMConstants.uam)
					|| ((Leg) pe).getMode().equals(UAMConstants.access + TransportMode.walk)
					|| ((Leg) pe).getMode().equals(UAMConstants.egress + TransportMode.walk)
					|| ((Leg) pe).getMode().equals(UAMConstants.access + TransportMode.bike)
					|| ((Leg) pe).getMode().equals(UAMConstants.egress + TransportMode.bike)
					|| ((Leg) pe).getMode().equals(UAMConstants.access + TransportMode.car)
					|| ((Leg) pe).getMode().equals(UAMConstants.egress + TransportMode.car))) {
				return UAMConstants.uam;
			}
		}
		// if the trip doesn't contain a uam leg,
		// fall back to the default identification method.
		return defaultModeIdentifier.identifyMainMode(tripElements);
	}
}