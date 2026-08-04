package net.bhl.matsim.uam.dmc;

import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.matsim.api.core.v01.population.PlanElement;
import org.matsim.api.core.v01.population.Person;
import org.matsim.contribs.discrete_mode_choice.components.estimators.AbstractTripRouterEstimator;
import org.matsim.contribs.discrete_mode_choice.model.DiscreteModeChoiceTrip;
import org.matsim.contribs.discrete_mode_choice.model.trip_based.candidates.TripCandidate;
import org.matsim.core.config.Config;
import org.matsim.core.config.ConfigGroup;
import org.matsim.core.router.TripRouter;
import org.matsim.core.utils.geometry.CoordUtils;
import org.matsim.core.utils.timing.TimeInterpretation;
import org.matsim.core.utils.timing.TimeTracker;
import org.matsim.facilities.ActivityFacilities;

import com.google.inject.Inject;

/**
 * Re-implements eqasim's Ile-de-France mode-choice utility formula (see
 * org.eqasim.core.simulation.mode_choice.utilities.estimators.{Car,Pt,Bike,Walk}UtilityEstimator
 * in eqasim-java -- not a runtime dependency of this project, ported by hand
 * because eqasim's compiled classes target a different MATSim core version
 * and are not classpath-compatible with this build), extended with a "uam"
 * row (rescaled from Fu, Rothfeld & Antoniou 2019). See the 2026-08-03
 * conversation / utility_model_config.yaml for the full derivation.
 *
 * V_mode = asc + betaTimeUtilPerMin * duration_min
 *          + betaCostUtilPerEur * (dist_km / referenceDistanceKm)^lambdaCostDistance
 *            * (costEurPerKm * dist_km)
 *
 * Distance is Euclidean (origin/destination activity coordinates), matching
 * eqasim's own convention (euclideanDistance_km), not routed distance.
 * Duration comes from actually routing the trip for this mode (via
 * AbstractTripRouterEstimator, MATSim's own trip-based-estimator base class
 * -- NOT from the `candidates` list DMC passes in, which is empty until an
 * estimator has already produced something for this trip; confirmed via a
 * real IndexOutOfBoundsException the first version of this class hit trying
 * to read candidates.get(0)).
 *
 * Parameters are read from the "idfUtilityModel" config module (a plain,
 * untyped ConfigGroup, registered via IdfUtilityModelConfigGroup so MATSim's
 * consistency checker accepts it) rather than hardcoded, so
 * utility_model_config.yaml stays the single source of truth -- see
 * baseline_brulin/matsim_config_utils.py's apply_idf_utility_model for how
 * that module gets populated.
 */
public class IdfStyleTripEstimator extends AbstractTripRouterEstimator {
	public static final String CONFIG_MODULE_NAME = "idfUtilityModel";

	private final TimeInterpretation timeInterpretation;
	private final double betaCostUtilPerEur;
	private final double referenceDistanceKm;
	private final double lambdaCostDistance;
	// mode -> [asc, betaTimeUtilPerMin, costEurPerKm]
	private final Map<String, double[]> modeParameters = new HashMap<>();

	@Inject
	public IdfStyleTripEstimator(TripRouter tripRouter, ActivityFacilities facilities,
			TimeInterpretation timeInterpretation, Config config) {
		super(tripRouter, facilities, timeInterpretation, Collections.emptyList());
		this.timeInterpretation = timeInterpretation;

		ConfigGroup module = config.getModules().get(CONFIG_MODULE_NAME);
		if (module == null) {
			throw new RuntimeException(
					"Missing '" + CONFIG_MODULE_NAME + "' config module -- required by IdfStyleTripEstimator");
		}

		this.betaCostUtilPerEur = Double.parseDouble(module.getValue("betaCostUtilPerEur"));
		this.referenceDistanceKm = Double.parseDouble(module.getValue("referenceDistanceKm"));
		this.lambdaCostDistance = Double.parseDouble(module.getValue("lambdaCostDistance"));

		for (Map.Entry<String, String> entry : module.getParams().entrySet()) {
			String key = entry.getKey();
			int dot = key.indexOf('.');
			if (dot < 0) {
				continue;
			}
			String mode = key.substring(0, dot);
			String field = key.substring(dot + 1);
			double[] params = modeParameters.computeIfAbsent(mode, m -> new double[3]);

			if (field.equals("asc")) {
				params[0] = Double.parseDouble(entry.getValue());
			} else if (field.equals("betaTimeUtilPerMin")) {
				params[1] = Double.parseDouble(entry.getValue());
			} else if (field.equals("costEurPerKm")) {
				params[2] = Double.parseDouble(entry.getValue());
			}
		}
	}

	// car_passenger is a deliberately fixed/captive exogenous segment in this
	// project (see build_uam_scenario.py's LIVE_MODES, which excludes it from
	// SubtourModeChoice's mutation set the same way) -- not a real utility
	// model row, so it has no idfUtilityModel entry. A TripConstraint-based
	// exclusion attempt (rejecting every mode but a trip's own initial one)
	// did not work: MATSim kept reporting the class-default tripConstraints
	// regardless of what config set, for reasons not fully pinned down (see
	// apply_dmc_config's docstring). This is the actual fix: car_passenger is
	// added to availableModes (so DMC estimates it as a real candidate for
	// every trip) and given the selector's own clamped max/min utility here
	// -- +700 when it's a trip's own initial mode (chosen with ~probability
	// 1), -700 otherwise (never chosen for any other trip). Matches
	// selector:MultinomialLogit's maximumUtility/minimumUtility (see
	// apply_dmc_config).
	private static final String CAR_PASSENGER_MODE = "car_passenger";
	private static final double CLAMPED_MAX_UTILITY = 700.0;
	private static final double CLAMPED_MIN_UTILITY = -700.0;

	@Override
	protected double estimateTrip(Person person, String mode, DiscreteModeChoiceTrip trip,
			List<TripCandidate> candidates, List<? extends PlanElement> elements) {
		if (mode.equals(CAR_PASSENGER_MODE)) {
			return trip.getInitialMode().equals(CAR_PASSENGER_MODE) ? CLAMPED_MAX_UTILITY : CLAMPED_MIN_UTILITY;
		}

		double[] params = modeParameters.get(mode);
		if (params == null) {
			throw new RuntimeException("IdfStyleTripEstimator has no idfUtilityModel parameters for mode '" + mode
					+ "' -- add " + mode + ".asc / " + mode + ".betaTimeUtilPerMin / " + mode
					+ ".costEurPerKm to the config");
		}
		double asc = params[0];
		double betaTimeUtilPerMin = params[1];
		double costEurPerKm = params[2];

		TimeTracker timeTracker = new TimeTracker(timeInterpretation);
		timeTracker.setTime(trip.getDepartureTime());
		timeTracker.addElements(elements);
		double durationMin = (timeTracker.getTime().seconds() - trip.getDepartureTime()) / 60.0;

		double distanceKm = 1e-3 * CoordUtils.calcEuclideanDistance(trip.getOriginActivity().getCoord(),
				trip.getDestinationActivity().getCoord());

		double dampedCostRate = betaCostUtilPerEur
				* Math.pow(Math.max(distanceKm, 1e-3) / referenceDistanceKm, lambdaCostDistance);
		double cost = costEurPerKm * distanceKm;

		return asc + betaTimeUtilPerMin * durationMin + dampedCostRate * cost;
	}
}
