package net.bhl.matsim.uam.theta;

import org.matsim.core.controler.AbstractModule;

/** Installs the theta exporter; the listener itself is gated by thetaExport.enabled. */
public final class ThetaExportModule extends AbstractModule {
	@Override
	public void install() {
		addControlerListenerBinding().to(ThetaExportListener.class);
	}
}
