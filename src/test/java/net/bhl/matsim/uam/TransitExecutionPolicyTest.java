package net.bhl.matsim.uam;

import net.bhl.matsim.uam.run.RunUAMScenario;
import org.junit.Test;
import org.matsim.core.config.Config;
import org.matsim.core.scenario.ScenarioUtils;

import static org.junit.Assert.assertEquals;

public class TransitExecutionPolicyTest {
    @Test
    public void controllerPreservesScenarioTransitPolicy() {
        for (boolean useTransit : new boolean[]{false, true}) {
            for (boolean simulateTransit : new boolean[]{false, true}) {
                RunUAMScenario.parseArguments(new String[]{"unused.xml"});
                Config config = RunUAMScenario.createConfig();
                config.transit().setUseTransit(useTransit);
                config.transit().setUsingTransitInMobsim(simulateTransit);
                RunUAMScenario.setScenario(ScenarioUtils.createScenario(config));
                RunUAMScenario.createControler();
                assertEquals(useTransit, config.transit().isUseTransit());
                assertEquals(simulateTransit, config.transit().isUsingTransitInMobsim());
            }
        }
    }
}
