package net.bhl.matsim.uam.router;

import java.util.List;
import java.util.Set;

import org.matsim.api.core.v01.TransportMode;
import org.matsim.api.core.v01.population.Leg;
import org.matsim.api.core.v01.population.PlanElement;
import org.matsim.core.router.AnalysisMainModeIdentifier;
import org.matsim.core.router.MainModeIdentifier;
import org.matsim.core.router.TripStructureUtils;

import net.bhl.matsim.uam.run.UAMConstants;

/**
 * Mode-share reporting identifier: a trip counts as mode M only if it really
 * contains a leg of mode M.
 *
 * Bound to AnalysisMainModeIdentifier (ModeStatsControlerListener's key) in
 * UAMModule. Deliberately NOT bound to MainModeIdentifier, which routing and
 * replanning consume -- changing that would change simulated behavior, not just
 * what gets reported.
 *
 * Why this exists (2026-09-21). The previous reporting identifier was
 * UAMMainModeIdentifier(new MainModeIdentifierImpl()). MainModeIdentifierImpl
 * predates the routingMode attribute: it returns the first leg whose mode does
 * not contain "transit_walk"/"non_network_walk". MATSim 2024.0 routes a pt trip
 * as [walk(routingMode=pt), pt, walk(routingMode=pt)] -- transit_walk was
 * removed years ago -- so every pt trip was reported as "walk", and Munich's
 * modestats.csv showed pt = 0.0 from the first replanned iteration onward, with
 * pt's real share folded into walk. Corsica was unaffected only because its DMC
 * availableModes excludes pt.
 *
 * Reading routingMode alone (TripStructureUtils.identifyMainMode, i.e. MATSim's
 * own RoutingModeMainModeIdentifier) fixes that but overcorrects, because
 * routingMode records what the router was ASKED for, not what it produced.
 * Where no itinerary exists, MATSim's routers return a plain walk leg while
 * keeping the requested routingMode -- the same silent fallback documented for
 * Corsica's pt in matsim_config_utils.apply_dmc_config, and for uam in the
 * beeline-walk teleport case. Measured on Munich design M12/M3/M39/M44,
 * iteration 1 selected plans: of 1181 trips with routingMode=pt, 298 (25%)
 * contain no pt leg at all; of 39 with routingMode=uam, 7 contain no uam leg.
 * Counting those would overstate real transit by a quarter and inflate the
 * headline uam share by 22% against every run recorded so far.
 *
 * So: a real uam leg still wins outright (identical uam column to before, which
 * keeps historical mean_uam_share/best_real_share comparable); otherwise take
 * routingMode, but demote pt/uam back to the legacy answer -- the walk the
 * agent actually performed -- when the matching leg is absent. car/bike/walk
 * legs always carry their own mode, so they are unaffected either way.
 */
public class UAMAnalysisMainModeIdentifier implements AnalysisMainModeIdentifier {
	/** Mirrors UAMMainModeIdentifier's own list; a trip holding any of these is a uam trip. */
	private static final Set<String> UAM_LEG_MODES = Set.of(
			UAMConstants.uam,
			UAMConstants.access + TransportMode.walk, UAMConstants.egress + TransportMode.walk,
			UAMConstants.access + TransportMode.bike, UAMConstants.egress + TransportMode.bike,
			UAMConstants.access + TransportMode.car, UAMConstants.egress + TransportMode.car);

	private final MainModeIdentifier legacyIdentifier;

	public UAMAnalysisMainModeIdentifier(final MainModeIdentifier legacyIdentifier) {
		this.legacyIdentifier = legacyIdentifier;
	}

	@Override
	public String identifyMainMode(List<? extends PlanElement> tripElements) {
		if (hasLegIn(tripElements, UAM_LEG_MODES)) {
			return UAMConstants.uam;
		}

		String routingMode = TripStructureUtils.identifyMainMode(tripElements);
		if (routingMode == null) {
			// identifyMainMode logs an error and returns null when a multi-leg
			// trip carries no routingMode at all; nothing better to do than the
			// pre-2026-09-21 answer.
			return legacyIdentifier.identifyMainMode(tripElements);
		}
		if (UAMConstants.uam.equals(routingMode)) {
			// No uam leg (checked above) -- a beeline-walk teleport fallback.
			return legacyIdentifier.identifyMainMode(tripElements);
		}
		if (TransportMode.pt.equals(routingMode) && !hasLegIn(tripElements, Set.of(TransportMode.pt))) {
			// pt requested, SwissRailRaptor found no itinerary, returned a walk.
			return legacyIdentifier.identifyMainMode(tripElements);
		}
		return routingMode;
	}

	private static boolean hasLegIn(List<? extends PlanElement> tripElements, Set<String> modes) {
		for (PlanElement planElement : tripElements) {
			if (planElement instanceof Leg && modes.contains(((Leg) planElement).getMode())) {
				return true;
			}
		}
		return false;
	}
}
