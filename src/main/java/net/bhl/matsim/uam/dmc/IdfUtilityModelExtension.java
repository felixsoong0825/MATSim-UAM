package net.bhl.matsim.uam.dmc;

import org.matsim.contribs.discrete_mode_choice.modules.AbstractDiscreteModeChoiceExtension;

/**
 * Registers IdfStyleTripEstimator under the name "IdfStyleTripScoring" so a
 * config can select it via DiscreteModeChoice's tripEstimator param (the
 * same mechanism that already selects the contrib's own built-in
 * "MATSimTripScoring") -- see baseline_brulin/matsim_config_utils.py's
 * apply_idf_utility_model. Installed alongside DiscreteModeChoiceModule in
 * RunUAMScenario.createControler().
 */
public class IdfUtilityModelExtension extends AbstractDiscreteModeChoiceExtension {
	public static final String TRIP_ESTIMATOR_NAME = "IdfStyleTripScoring";

	@Override
	protected void installExtension() {
		bindTripEstimator(TRIP_ESTIMATOR_NAME).to(IdfStyleTripEstimator.class);
	}
}
