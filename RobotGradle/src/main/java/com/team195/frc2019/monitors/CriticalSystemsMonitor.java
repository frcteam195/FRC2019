package com.team195.frc2019.monitors;

import com.team195.frc2019.Constants;
import com.team195.frc2019.SubsystemManager;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.reporters.MessageLevel;
import com.team195.frc2019.subsystems.Subsystem;
import com.team195.lib.util.CriticalSystemStatus;
import com.team195.lib.util.ThreadRateControl;

import java.util.ArrayList;

public class CriticalSystemsMonitor {
	private static final int MIN_SYSTEM_MONITOR_LOOP_MS = 250;

	private static CriticalSystemsMonitor instance = null;

	private boolean runThread = true;
	private SubsystemManager subsystemManager = SubsystemManager.getInstance();

	private CriticalSystemsMonitor() {
		ThreadRateControl threadRateControl = new ThreadRateControl();
		Thread t = new Thread(() -> {
			threadRateControl.start();
			while (runThread) {
				subsystemManager.checkSubsystemFaulted();
				threadRateControl.doRateControl(MIN_SYSTEM_MONITOR_LOOP_MS);
			}
		});
		t.setPriority(Constants.kCriticalSystemsMonitorThreadPriority);
		t.start();
	}

	public static CriticalSystemsMonitor getInstance() {
		if(instance == null) {
			try {
				instance = new CriticalSystemsMonitor();
			} catch (Exception ex) {
				ConsoleReporter.report(ex, MessageLevel.DEFCON1);
			}
		}

		return instance;
	}
}
