package net.bhl.matsim.uam.dmc;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.matsim.api.core.v01.Coord;
import org.matsim.api.core.v01.population.Leg;
import org.matsim.api.core.v01.population.PlanElement;
import org.matsim.api.core.v01.population.Person;
import org.matsim.contribs.discrete_mode_choice.components.estimators.AbstractTripRouterEstimator;
import org.matsim.contribs.discrete_mode_choice.model.DiscreteModeChoiceTrip;
import org.matsim.contribs.discrete_mode_choice.model.trip_based.candidates.TripCandidate;
import org.matsim.core.config.Config;
import org.matsim.core.config.ConfigGroup;
import org.matsim.core.controler.MatsimServices;
import org.matsim.core.router.TripRouter;
import org.matsim.core.utils.geometry.CoordUtils;
import org.matsim.core.utils.timing.TimeInterpretation;
import org.matsim.core.utils.timing.TimeTracker;
import org.matsim.facilities.ActivityFacilities;
import org.matsim.pt.routes.TransitPassengerRoute;

import com.google.inject.Inject;
import com.google.inject.Provider;

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
 *
 * Optional candidate logging (2026-08-04): if the same config module sets
 * "candidateLogPath" (and, optionally, "candidateLogStartIteration", default
 * the last iteration of the run -- i.e. a single-iteration snapshot unless
 * the caller explicitly widens it), every real (non-car_passenger)
 * estimateTrip() call made from that iteration ONWARD (inclusive, through
 * the run's end) appends a row -- the same duration/distance/cost this class
 * already computes internally, just persisted instead of discarded -- to
 * that CSV. A multi-iteration window (not just the final iteration) matters
 * here because DMC's real per-round output oscillates in a band rather than
 * settling to a point -- same reason baseline_brulin/evaluate_subset.py
 * averages the aggregate mode-share trajectory over a tail window instead of
 * reading a single final iteration. Off (zero behavior/perf change) unless
 * candidateLogPath is explicitly set; see cflm_space_mapping's
 * cflm_uam.dmc_extraction for the reader that turns this log into the
 * ground_times/access/egress shape the CFLM MILP already consumes, closing
 * the gap where those were previously a static, MATSim-run-independent skim.
 *
 * Nested pt time split (2026-08-05, Munich port): unlike car/bike/walk/uam,
 * whose utility_model_config.yaml entry has a single flat
 * beta_time_util_per_min scalar, Munich's "pt" entry splits it into
 * in_vehicle/wait_and_walk components (real transit trips genuinely feel
 * different per minute spent actually moving vs waiting/walking -- standard
 * in transit mode-choice literature). See modeParameters' own comment and
 * splitInVehicleAndWaitAndWalkMinutes() below for how that's implemented;
 * apply_idf_utility_model (matsim_config_utils.py) is the Python-side writer
 * that turns the yaml's nested dict into this class's
 * "{mode}.betaTimeUtilPerMin" / "{mode}.betaTimeUtilPerMinWaitAndWalk" pair
 * of config keys.
 */
public class IdfStyleTripEstimator extends AbstractTripRouterEstimator {
	public static final String CONFIG_MODULE_NAME = "idfUtilityModel";

	private final TimeInterpretation timeInterpretation;
	private final double betaCostUtilPerEur;
	private final double referenceDistanceKm;
	private final double lambdaCostDistance;
	// mode -> [asc, betaTimeUtilPerMin, costEurPerKm, betaTimeUtilPerMinWaitAndWalk]
	//
	// index 3 (2026-08-05, Munich pt port) is NaN for every mode except ones
	// whose utility_model_config.yaml entry has a nested
	// beta_time_util_per_min (currently just Munich's "pt": in_vehicle vs
	// wait_and_walk -- see apply_idf_utility_model's docstring in
	// matsim_config_utils.py, the Python config writer that populates these
	// two keys from that yaml). When index 3 is NaN, estimateTrip() uses the
	// old single-flat-beta formula unchanged (index 1 * total routed
	// duration) -- car/bike/walk/uam (and any future flat-beta mode) are
	// byte-for-byte unaffected by this change. When index 3 is set, index 1
	// is instead interpreted as the IN-VEHICLE-only rate and index 3 as the
	// rate for every other routed-trip minute (access/egress walk + waiting
	// for the vehicle, folded together) -- see
	// splitInVehicleAndWaitAndWalkMinutes(). Corsica's own pt entry has a
	// 3-way split (in_vehicle/waiting/access_egress) that this does NOT
	// reproduce -- irrelevant in practice since Corsica's DMC availableModes
	// still excludes "pt" entirely (see apply_dmc_config's docstring), so
	// this code path is never exercised for Corsica.
	private final Map<String, double[]> modeParameters = new HashMap<>();

	// null candidateLogPath = logging disabled (the default for every existing
	// config/run). candidateLogStartIteration resolved lazily against the
	// real last iteration the first time it's needed, since Config's
	// controler module is available at construction time but MatsimServices
	// isn't yet.
	//
	// The lock/writer/resolved-iteration are STATIC, not per-instance: DMC's
	// replanning is multi-threaded and this class has no @Singleton scope,
	// so Guice hands out a fresh IdfStyleTripEstimator per worker thread --
	// confirmed the hard way, an earlier per-instance-field version produced
	// a corrupted candidate log (interleaved/truncated CSV rows) because
	// each instance opened its own independent FileWriter on the same path.
	// A static lock+writer serializes every real write across every
	// instance/thread in the JVM, which is what's actually needed here.
	private final String candidateLogPath;
	private final Integer candidateLogStartIterationOverride;
	private final Provider<MatsimServices> matsimServicesProvider;
	private static final Object CANDIDATE_LOG_LOCK = new Object();
	private static volatile BufferedWriter candidateLogWriter;
	private static volatile int resolvedCandidateLogStartIteration = Integer.MIN_VALUE;

	@Inject
	public IdfStyleTripEstimator(TripRouter tripRouter, ActivityFacilities facilities,
			TimeInterpretation timeInterpretation, Config config, Provider<MatsimServices> matsimServicesProvider) {
		super(tripRouter, facilities, timeInterpretation, Collections.emptyList());
		this.timeInterpretation = timeInterpretation;
		this.matsimServicesProvider = matsimServicesProvider;

		ConfigGroup module = config.getModules().get(CONFIG_MODULE_NAME);
		if (module == null) {
			throw new RuntimeException(
					"Missing '" + CONFIG_MODULE_NAME + "' config module -- required by IdfStyleTripEstimator");
		}

		this.betaCostUtilPerEur = Double.parseDouble(module.getValue("betaCostUtilPerEur"));
		this.referenceDistanceKm = Double.parseDouble(module.getValue("referenceDistanceKm"));
		this.lambdaCostDistance = Double.parseDouble(module.getValue("lambdaCostDistance"));

		this.candidateLogPath = module.getValue("candidateLogPath");
		String logStartIterationValue = module.getValue("candidateLogStartIteration");
		this.candidateLogStartIterationOverride = logStartIterationValue != null
				? Integer.parseInt(logStartIterationValue) : null;

		for (Map.Entry<String, String> entry : module.getParams().entrySet()) {
			String key = entry.getKey();
			int dot = key.indexOf('.');
			if (dot < 0) {
				continue;
			}
			String mode = key.substring(0, dot);
			String field = key.substring(dot + 1);
			double[] params = modeParameters.computeIfAbsent(mode, m -> new double[]{0.0, 0.0, 0.0, Double.NaN});

			if (field.equals("asc")) {
				params[0] = Double.parseDouble(entry.getValue());
			} else if (field.equals("betaTimeUtilPerMin")) {
				params[1] = Double.parseDouble(entry.getValue());
			} else if (field.equals("costEurPerKm")) {
				params[2] = Double.parseDouble(entry.getValue());
			} else if (field.equals("betaTimeUtilPerMinWaitAndWalk")) {
				params[3] = Double.parseDouble(entry.getValue());
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
		double betaTimeUtilPerMinWaitAndWalk = params[3];

		TimeTracker timeTracker = new TimeTracker(timeInterpretation);
		timeTracker.setTime(trip.getDepartureTime());
		timeTracker.addElements(elements);
		double durationMin = (timeTracker.getTime().seconds() - trip.getDepartureTime()) / 60.0;

		double distanceKm = 1e-3 * CoordUtils.calcEuclideanDistance(trip.getOriginActivity().getCoord(),
				trip.getDestinationActivity().getCoord());

		double dampedCostRate = betaCostUtilPerEur
				* Math.pow(Math.max(distanceKm, 1e-3) / referenceDistanceKm, lambdaCostDistance);
		double cost = costEurPerKm * distanceKm;

		double timeUtility;
		if (Double.isNaN(betaTimeUtilPerMinWaitAndWalk)) {
			// Every mode except a split-configured one (currently only
			// Munich's "pt", see modeParameters' own comment above): a
			// single flat rate over the whole routed trip duration, exactly
			// as before this split was added -- zero behavior change.
			timeUtility = betaTimeUtilPerMin * durationMin;
		} else {
			double[] split = splitInVehicleAndWaitAndWalkMinutes(elements, trip.getDepartureTime());
			double inVehicleMin = split[0];
			double waitAndWalkMin = split[1];
			timeUtility = betaTimeUtilPerMin * inVehicleMin + betaTimeUtilPerMinWaitAndWalk * waitAndWalkMin;
		}
		double utility = asc + timeUtility + dampedCostRate * cost;

		if (candidateLogPath != null) {
			maybeLogCandidate(person, mode, trip, durationMin, distanceKm, cost, utility);
		}

		return utility;
	}

	// 2026-08-05 (Munich pt port): splits a routed trip's elements into
	// in-vehicle minutes (actually moving on a transit vehicle) vs
	// wait-and-walk minutes (access/egress walk legs, transfer walks, AND
	// time spent waiting at a stop for the boarded vehicle to actually
	// depart) -- only called for modes with a configured
	// betaTimeUtilPerMinWaitAndWalk (see modeParameters' own comment).
	//
	// A real pt leg's Route is a TransitPassengerRoute (see
	// ch.sbb.matsim.routing.pt.raptor.RaptorUtils.convertRouteToLegs, the
	// SwissRailRaptor code that actually builds these elements) -- checking
	// `instanceof TransitPassengerRoute` rather than matching on a mode-name
	// string is deliberate: SwissRailRaptor's own
	// SwissRailRaptorRoutingModule.findCoordinate() uses the exact same
	// instanceof check to distinguish real transit-vehicle legs from walk
	// legs, and the individual per-leg mode string a RaptorRoute part ends
	// up carrying is not guaranteed to be the generic "pt" (it can be the
	// underlying schedule mode, e.g. "bus"/"tram"/"rail"/"subway" --
	// confirmed from RaptorRoute.addPt's own call site), so a string check
	// would be fragile where this one is not.
	//
	// A TransitPassengerRoute's own travel time (leg arrival - leg start)
	// bundles together any wait-for-the-vehicle time AND the actual
	// in-vehicle ride time -- RaptorUtils.convertRouteToLegs sets
	// ptRoute.setBoardingTime(part.boardingTime) separately from the leg's
	// own departure time specifically so callers can recover this split:
	// wait = max(0, boardingTime - legStartTime), in-vehicle = legTravelTime
	// - wait. "pt interaction" stage activities between legs (zero
	// scheduled duration, agents pass through them immediately) are not
	// separately accounted for -- any real transfer wait is already folded
	// into the NEXT pt leg's own boardingTime-vs-legStart gap, not the
	// activity's own (zero) duration, so walking Legs only (via TimeTracker,
	// for exact consistency with durationMin's own time bookkeeping above)
	// is sufficient.
	private double[] splitInVehicleAndWaitAndWalkMinutes(List<? extends PlanElement> elements,
			double tripDepartureTime) {
		double inVehicleSeconds = 0.0;
		double waitAndWalkSeconds = 0.0;

		TimeTracker timeTracker = new TimeTracker(timeInterpretation);
		timeTracker.setTime(tripDepartureTime);
		for (PlanElement pe : elements) {
			if (pe instanceof Leg leg) {
				double legStart = timeTracker.getTime().seconds();
				timeTracker.addElement(pe);
				double legDuration = timeTracker.getTime().seconds() - legStart;

				if (leg.getRoute() instanceof TransitPassengerRoute ptRoute) {
					double boardingTime = ptRoute.getBoardingTime().orElse(legStart);
					double waitSeconds = Math.max(0.0, boardingTime - legStart);
					double inVehicleSecondsThisLeg = Math.max(0.0, legDuration - waitSeconds);
					waitAndWalkSeconds += waitSeconds;
					inVehicleSeconds += inVehicleSecondsThisLeg;
				} else {
					waitAndWalkSeconds += legDuration;
				}
			} else {
				timeTracker.addElement(pe);
			}
		}

		return new double[] { inVehicleSeconds / 60.0, waitAndWalkSeconds / 60.0 };
	}

	private int resolveCandidateLogStartIteration() {
		if (candidateLogStartIterationOverride != null) {
			return candidateLogStartIterationOverride;
		}
		return matsimServicesProvider.get().getConfig().controller().getLastIteration();
	}

	private void maybeLogCandidate(Person person, String mode, DiscreteModeChoiceTrip trip, double durationMin,
			double distanceKm, double cost, double utility) {
		if (resolvedCandidateLogStartIteration == Integer.MIN_VALUE) {
			synchronized (CANDIDATE_LOG_LOCK) {
				if (resolvedCandidateLogStartIteration == Integer.MIN_VALUE) {
					resolvedCandidateLogStartIteration = resolveCandidateLogStartIteration();
				}
			}
		}

		// >= not ==: 2026-08-04, logs every iteration from the (resolved)
		// start iteration through the run's end, not just one snapshot --
		// DMC's real per-round output oscillates in a band rather than
		// settling to a point (same reason evaluate_subset.py averages the
		// aggregate mode-share trajectory over a tail window instead of
		// reading a single final iteration), so the candidate log needs the
		// same tail-window averaging treatment, not just a cheaper iteration
		// count.
		Integer currentIteration = matsimServicesProvider.get().getIterationNumber();
		if (currentIteration == null || currentIteration < resolvedCandidateLogStartIteration) {
			return;
		}

		String originFacility = trip.getOriginActivity().getFacilityId() != null
				? trip.getOriginActivity().getFacilityId().toString()
				: trip.getOriginActivity().getType();
		String destinationFacility = trip.getDestinationActivity().getFacilityId() != null
				? trip.getDestinationActivity().getFacilityId().toString()
				: trip.getDestinationActivity().getType();

		// 2026-08-05 (Munich port): origin_facility/destination_facility fall
		// back to the bare activity TYPE ("home"/"education"/...) whenever a
		// scenario has no real facilities layer at all (Munich's real MITO
		// population never sets facility ids) -- that collapses every trip
		// sharing an activity-type pair into one fake OD group, breaking
		// cflm_uam.dmc_extraction's real-first ground_times merge (confirmed:
		// 0/5349 real observations matched on Munich's first real test,
		// vs Corsica's 81%). Logging the real coordinates here too lets the
		// Python side fall back to a coordinate match (mirroring
		// theta_correction.py's own nearest-zone lookup against real leg
		// coordinates) instead of a facility-id string match whenever that
		// string is just an activity type -- unaffected for Corsica, which
		// still has real facility ids to match on directly.
		Coord originCoord = trip.getOriginActivity().getCoord();
		Coord destinationCoord = trip.getDestinationActivity().getCoord();

		String line = String.join(",", person.getId().toString(), originFacility, destinationFacility, mode,
				String.valueOf(trip.getDepartureTime()), String.valueOf(durationMin), String.valueOf(distanceKm),
				String.valueOf(cost), String.valueOf(utility), String.valueOf(originCoord.getX()),
				String.valueOf(originCoord.getY()), String.valueOf(destinationCoord.getX()),
				String.valueOf(destinationCoord.getY()));

		synchronized (CANDIDATE_LOG_LOCK) {
			try {
				BufferedWriter writer = getCandidateLogWriter();
				writer.write(line);
				writer.newLine();
			} catch (IOException e) {
				throw new UncheckedIOException(e);
			}
		}
	}

	private BufferedWriter getCandidateLogWriter() throws IOException {
		if (candidateLogWriter == null) {
			candidateLogWriter = new BufferedWriter(new FileWriter(candidateLogPath));
			candidateLogWriter.write("person_id,origin_facility,destination_facility,mode,departure_time_s,"
					+ "duration_min,distance_km,cost_eur,utility,origin_x,origin_y,destination_x,destination_y");
			candidateLogWriter.newLine();
			BufferedWriter writerToClose = candidateLogWriter;
			Runtime.getRuntime().addShutdownHook(new Thread(() -> {
				try {
					writerToClose.flush();
					writerToClose.close();
				} catch (IOException e) {
					// best-effort flush on shutdown; nothing more useful to do here
				}
			}));
		}
		return candidateLogWriter;
	}
}
