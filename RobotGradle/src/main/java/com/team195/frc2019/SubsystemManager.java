package com.team195.frc2019;

import com.team195.frc2019.loops.ILooper;
import com.team195.frc2019.loops.Loop;
import com.team195.frc2019.loops.Looper;
import com.team195.frc2019.monitors.CriticalSystemsMonitor;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.reporters.MessageLevel;
import com.team195.frc2019.subsystems.Subsystem;
import com.team195.lib.util.CriticalSystemStatus;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Used to reset, start, stop, and update all subsystems at once
 */
public class SubsystemManager implements ILooper {

	private static SubsystemManager instance = null;

	private static ArrayList<Subsystem> mAllSubsystems = new ArrayList<>();
	private List<Loop> mLoops = new ArrayList<>();

	private SubsystemManager() {

	}

	public static SubsystemManager getInstance(Subsystem... subsystems) {
		if(instance == null) {
			try {
				instance = new SubsystemManager();
			} catch (Exception ex) {
				ConsoleReporter.report(ex, MessageLevel.DEFCON1);
			}
		}

		mAllSubsystems.addAll(Arrays.asList(subsystems));

		return instance;
	}

	public boolean checkSubsystemFaulted() {
		ArrayList<Boolean> retVals = new ArrayList<>();
		mAllSubsystems.forEach((s) -> retVals.add(s.isSystemFaulted()));
		for (boolean b : retVals) {
			if (b)
				return true;
		}
		return false;
	}

	public boolean checkSystemsPassDiagnostics() {
		ArrayList<Boolean> retVals = new ArrayList<>();
		mAllSubsystems.forEach((s) -> retVals.add(s.runDiagnostics()));
		for (boolean b : retVals) {
			if (!b)
				return false;
		}
		return true;
	}

	public String generateReport() {
		StringBuilder sb = new StringBuilder();
		mAllSubsystems.forEach((s) -> sb.append(s.generateReport()));
		return sb.toString();
	}

	public void stop() {
		mAllSubsystems.forEach(Subsystem::stop);
	}

	private class EnabledLoop implements Loop {

		@Override
		public void onStart(double timestamp) {
			mLoops.forEach((l) -> l.onStart(timestamp));
		}

		@Override
		public void onLoop(double timestamp) {
			mAllSubsystems.forEach(Subsystem::readPeriodicInputs);
			mLoops.forEach((l) -> l.onLoop(timestamp));
			mAllSubsystems.forEach(Subsystem::writePeriodicOutputs);
		}

		@Override
		public void onStop(double timestamp) {
			mLoops.forEach((l) -> l.onStop(timestamp));
		}
	}

	private class DisabledLoop implements Loop {

		@Override
		public void onStart(double timestamp) {

		}

		@Override
		public void onLoop(double timestamp) {
			mAllSubsystems.forEach(Subsystem::readPeriodicInputs);
			mAllSubsystems.forEach(Subsystem::writePeriodicOutputs);
		}

		@Override
		public void onStop(double timestamp) {

		}
	}

	public void registerEnabledLoops(Looper enabledLooper) {
		mAllSubsystems.forEach((s) -> s.registerEnabledLoops(this));
		enabledLooper.register(new EnabledLoop());
	}

	public void registerDisabledLoops(Looper disabledLooper) {
		disabledLooper.register(new DisabledLoop());
	}

	@Override
	public void register(Loop loop) {
		mLoops.add(loop);
	}
}
