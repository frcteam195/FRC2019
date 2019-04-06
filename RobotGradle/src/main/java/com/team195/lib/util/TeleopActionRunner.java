package com.team195.lib.util;

import com.team195.frc2019.auto.autonomy.AutomatedAction;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.reporters.MessageLevel;
import com.team195.frc2019.subsystems.Subsystem;
import com.team254.lib.util.CrashTrackingRunnable;
import edu.wpi.first.wpilibj.Notifier;

import java.util.*;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.ReentrantLock;

public class TeleopActionRunner {
	private static LinkedHashSet<AutomatedAction> mActionList = new LinkedHashSet<>();

	public static void processActions() {
		try {
			if (mActionList.size() > 0) {
				mActionList.forEach((action) -> {
					if (!action.isStarted())
						action.start();
					action.update();
				});
				mActionList.removeIf((action) -> {
					boolean finished = false;
					ConsoleReporter.report(action.getClass().getSimpleName() + " Action Running!", MessageLevel.INFO);
					if (action.isFinished()) {
						finished = true;
						action.done();
					}
					return finished;
				});
			}
		} catch (Exception ex) {
			ConsoleReporter.report(ex);
		}
	}

	public static boolean runAction(AutomatedAction action) {
		try {
			if (mActionList.size() > 0) {
				mActionList.removeIf((xAction) -> {
					for (Subsystem xSubsystem : xAction.getRequiredSubsystems()) {
						if (action.getRequiredSubsystems().contains(xSubsystem)) {
							action.purgeActions();
							return true;
						}
					}
					return false;
				});
			}
			mActionList.add(action);
		} catch (Exception ex) {
			ConsoleReporter.report(ex);
			return false;
		}

		return true;
	}
}
