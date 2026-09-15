package net.bhl.matsim.uam.theta;

import org.matsim.core.config.Config;
import org.matsim.core.config.ReflectiveConfigGroup;

/** Configuration for exporting the in-memory car link travel-time table. */
public final class ThetaExportConfigGroup extends ReflectiveConfigGroup {
	public static final String GROUP_NAME = "thetaExport";

	private static final String ENABLED = "enabled";
	private static final String FIRST_ITERATION = "firstIteration";
	private static final String LAST_ITERATION = "lastIteration";
	private static final String FILE_NAME = "fileName";
	private static final String MODE = "mode";

	private boolean enabled = false;
	private int firstIteration = 0;
	private int lastIteration = -1;
	private String fileName = "theta.csv.gz";
	private String mode = "car";

	public ThetaExportConfigGroup() {
		super(GROUP_NAME);
	}

	public static ThetaExportConfigGroup get(Config config) {
		return (ThetaExportConfigGroup) config.getModules().get(GROUP_NAME);
	}

	@StringGetter(ENABLED)
	public boolean isEnabled() {
		return enabled;
	}

	@StringSetter(ENABLED)
	public void setEnabled(boolean enabled) {
		this.enabled = enabled;
	}

	@StringGetter(FIRST_ITERATION)
	public int getFirstIteration() {
		return firstIteration;
	}

	@StringSetter(FIRST_ITERATION)
	public void setFirstIteration(int firstIteration) {
		this.firstIteration = firstIteration;
	}

	@StringGetter(LAST_ITERATION)
	public int getLastIteration() {
		return lastIteration;
	}

	@StringSetter(LAST_ITERATION)
	public void setLastIteration(int lastIteration) {
		this.lastIteration = lastIteration;
	}

	@StringGetter(FILE_NAME)
	public String getFileName() {
		return fileName;
	}

	@StringSetter(FILE_NAME)
	public void setFileName(String fileName) {
		this.fileName = fileName;
	}

	@StringGetter(MODE)
	public String getMode() {
		return mode;
	}

	@StringSetter(MODE)
	public void setMode(String mode) {
		this.mode = mode;
	}

	public boolean exportsIteration(int iteration) {
		return enabled && iteration >= firstIteration && (lastIteration < 0 || iteration <= lastIteration);
	}
}
