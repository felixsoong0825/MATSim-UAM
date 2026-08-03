package net.bhl.matsim.uam.dmc;

import org.matsim.core.config.ConfigGroup;

/**
 * Trivial named ConfigGroup for the "idfUtilityModel" module (see
 * IdfStyleTripEstimator) -- exists only so MATSim's
 * UnmaterializedConfigGroupChecker (which flags any module whose parsed
 * runtime class is literally the generic ConfigGroup base class, not a
 * registered subclass) doesn't reject it. Always registered in
 * RunUAMScenario, whether or not a given run's config actually uses
 * IdfStyleTripScoring -- same pattern as UAMConfigGroup/DvrpConfigGroup/
 * DiscreteModeChoiceConfigGroup, which are also unconditionally registered.
 * No typed fields: IdfStyleTripEstimator reads its params generically via
 * getValue/getParams, same as before.
 */
public class IdfUtilityModelConfigGroup extends ConfigGroup {
	public IdfUtilityModelConfigGroup() {
		super(IdfStyleTripEstimator.CONFIG_MODULE_NAME);
	}
}
