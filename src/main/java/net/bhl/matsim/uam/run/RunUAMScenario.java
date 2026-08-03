package net.bhl.matsim.uam.run;

import ch.sbb.matsim.routing.pt.raptor.SwissRailRaptorModule;
import net.bhl.matsim.uam.config.UAMConfigGroup;
import net.bhl.matsim.uam.dmc.IdfUtilityModelConfigGroup;
import net.bhl.matsim.uam.dmc.IdfUtilityModelExtension;
import net.bhl.matsim.uam.qsim.UAMQSimModule;
import net.bhl.matsim.uam.qsim.UAMSpeedModule;
import org.matsim.api.core.v01.Scenario;
import org.matsim.contribs.discrete_mode_choice.modules.DiscreteModeChoiceModule;
import org.matsim.contribs.discrete_mode_choice.modules.config.DiscreteModeChoiceConfigGroup;
import org.matsim.contrib.dvrp.run.DvrpConfigGroup;
import org.matsim.contrib.dvrp.run.DvrpModule;
import org.matsim.core.config.CommandLine;
import org.matsim.core.config.CommandLine.ConfigurationException;
import org.matsim.core.config.Config;
import org.matsim.core.config.ConfigUtils;
import org.matsim.core.config.groups.QSimConfigGroup.StarttimeInterpretation;
import org.matsim.core.config.groups.ScoringConfigGroup.ActivityParams;
import org.matsim.core.controler.Controler;
import org.matsim.core.scenario.ScenarioUtils;

import com.google.common.collect.ImmutableSet;

/**
 * The RunUAMScenario program start a MATSim run including Urban Air Mobility
 * capabilities.
 *
 * @author balacmi (Milos Balac), RRothfeld (Raoul Rothfeld)
 */
public class RunUAMScenario {

	private static UAMConfigGroup uamConfigGroup;
	private static CommandLine cmd;
	private static String path;
	private static Config config;
	private static Controler controler;
	private static Scenario scenario;

	public static void main(String[] args) {
		parseArguments(args);
		setConfig(path);
		createScenario();
		createControler().run();
	}

	public static void parseArguments(String[] args) {
		try {
			cmd = new CommandLine.Builder(args).allowOptions("config-path", "use-charging").build();

			if (cmd.hasOption("config-path"))
				path = cmd.getOption("config-path").get();
			else
				path = args[0];
		} catch (ConfigurationException e) {
			e.printStackTrace();
		}
		uamConfigGroup = new UAMConfigGroup();
	}

	public static Config createConfig() {
		return config = ConfigUtils.createConfig(uamConfigGroup, new DvrpConfigGroup(),
				new DiscreteModeChoiceConfigGroup(), new IdfUtilityModelConfigGroup());
	}

	public static Config setConfig(String path) {
		return config = ConfigUtils.loadConfig(path, uamConfigGroup, new DvrpConfigGroup(),
				new DiscreteModeChoiceConfigGroup(), new IdfUtilityModelConfigGroup());
	}

	public static Scenario createScenario() {
		scenario = ScenarioUtils.createScenario(config);
		ScenarioUtils.loadScenario(scenario);
		return scenario;
	}

	public static Scenario setScenario(Scenario scenario) {
		return RunUAMScenario.scenario = scenario;
	}

	public static Controler createControler() {
		try {
			cmd.applyConfiguration(config);
		} catch (ConfigurationException e) {
			e.printStackTrace();
		}

		controler = new Controler(scenario);

		controler.addOverridingModule(new DvrpModule());

		// Installs the discrete_mode_choice contrib's own Guice bindings (its
		// classes were already on the classpath as a pom.xml dependency, but
		// were never wired in -- this is what previously caused "PlanSelector
		// is not explicitly bound" when a config tried to use
		// strategyName=DiscreteModeChoice / planSelectorForRemoval=
		// NonSelectedPlanSelector). Safe to always install: it only adds a
		// named "DiscreteModeChoice" strategy provider and a conditional
		// NonSelectedPlanSelector binding (only activated if the config's
		// planSelectorForRemoval is actually set to that string) -- inert for
		// scenarios that don't reference either.
		controler.addOverridingModule(new DiscreteModeChoiceModule());
		// Registers "IdfStyleTripScoring" as a selectable tripEstimator name
		// alongside the contrib's own built-in "MATSimTripScoring" -- inert
		// unless a config's DiscreteModeChoice.tripEstimator actually
		// references it (see net.bhl.matsim.uam.dmc.IdfStyleTripEstimator).
		controler.addOverridingModule(new IdfUtilityModelExtension());

		controler.addOverridingModule(new UAMModule(config));
		controler.addOverridingQSimModule(new UAMSpeedModule());
		controler.addOverridingModule(new SwissRailRaptorModule());

		controler.configureQSimComponents(configurator -> {
			UAMQSimModule.activateModes().configure(configurator);
		});

		controler.getConfig().transit().setUseTransit(true);
		controler.getConfig().transit().setUsingTransitInMobsim(true);
		controler.getConfig().qsim().setSimStarttimeInterpretation(StarttimeInterpretation.onlyUseStarttime);
		controler.getConfig().qsim().setStartTime(0.0);

		DvrpConfigGroup.get(config).networkModes = ImmutableSet.of("uam");
		// (The 3 config.scoring().addModeParams(new ModeParams(...)) calls that used
		// to be here for access_uam_car/egress_uam_car/uam were removed: they ran
		// AFTER config loading and silently overwrote whatever
		// marginalUtilityOfTraveling_util_hr the XML config had already set for
		// exactly these 3 modes with ModeParams's bare-constructor default (matching
		// MATSim's generic "unrecognized mode" default, -6.0 -- same value seen on
		// "ride"/"other"). build_uam_scenario.py's merge_uam_into_base_config already
		// sets marginalUtilityOfTraveling_util_hr correctly for all 7 UAM_MODE_PARAMS
		// modes (including these 3) before this ever runs, so these lines were purely
		// redundant-and-harmful once that Python-side mechanism existed. Confirmed via
		// a direct diff of the merged input config (-1.0) vs MATSim's own
		// post-load output config (-6.0), with an "mode parameters for mode uam were
		// just overwritten" log line pointing straight at this call site.
		config.scoring()
				.addActivityParams(new ActivityParams("uam_interaction").setScoringThisActivityAtAll(false));

		config.controller().setWriteEventsInterval(1);
		
		return controler;
	}
}
