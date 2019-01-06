package com.team195.frc2019.subsystems;

import com.team195.frc2019.Constants;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.reporters.MessageLevel;
import com.team195.lib.util.CriticalSystemStatus;
import com.team195.lib.util.ThreadRateControl;

import java.util.ArrayList;

public class CriticalSystemsMonitor extends Thread {
	private static final int MIN_SYSTEM_MONITOR_LOOP_MS = 250;

	private static CriticalSystemsMonitor instance = null;

	private boolean runThread = true;
	private ThreadRateControl threadRateControl = new ThreadRateControl();
	private ArrayList<CriticalSystemStatus> mSystemArr;

	private CriticalSystemsMonitor(ArrayList<Subsystem> subsystems) {
		super();
		super.setPriority(Constants.kCriticalSystemsMonitorThreadPriority);
		mSystemArr = new ArrayList<CriticalSystemStatus>();
		for (Subsystem cs : subsystems) {
			if (cs instanceof CriticalSystemStatus)
				mSystemArr.add((CriticalSystemStatus) cs);
		}
	}

	public static CriticalSystemsMonitor getInstance(ArrayList<Subsystem> subsystems) {
		if(instance == null) {
			try {
				instance = new CriticalSystemsMonitor(subsystems);
			} catch (Exception ex) {
				ConsoleReporter.report(ex, MessageLevel.DEFCON1);
			}
		}

		return instance;
	}

	@Override
	public void start() {
		runThread = true;
		if (!super.isAlive())
			super.start();
	}

	@Override
	public void run() {
		threadRateControl.start();
		while (runThread) {
			for (CriticalSystemStatus css : mSystemArr)
				css.isSystemFaulted();

			threadRateControl.doRateControl(MIN_SYSTEM_MONITOR_LOOP_MS);
		}
	}
}
