package com.team195.lib.util;

public interface DiagnosableSubsystem {
	/**
	 * Method to diagnose a subsystem
	 * @return Returns true if the system passes all tests
	 */
	 boolean runDiagnostics();
}
