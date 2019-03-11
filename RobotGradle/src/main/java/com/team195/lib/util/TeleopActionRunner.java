package com.team195.lib.util;

import com.team195.frc2019.Constants;
import com.team195.frc2019.auto.actions.Action;
import com.team195.frc2019.auto.autonomy.AutomatedAction;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.reporters.MessageLevel;
import com.team195.frc2019.subsystems.Subsystem;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

import java.util.*;
import java.util.concurrent.locks.ReentrantLock;

public class TeleopActionRunner {
	private static double m_update_rate = 1.0 / 50.0;    //20ms update rate
	private static HashSet<AutomatedAction> mActionList = new HashSet<>();
	private static ReentrantLock mActionLock = new ReentrantLock();

	static {
		Thread mRunnerThread = new Thread(() -> {
			ThreadRateControl threadRateControl = new ThreadRateControl();
			threadRateControl.start();

			while (true) {
				try {
					mActionLock.lock();
					mActionList.forEach((action) -> {
						if (!action.isStarted())
							action.start();
						action.update();
					});
					mActionList.removeIf((action) -> {
						boolean finished = false;
						if (action.isFinished()) {
							finished = true;
							action.done();
						}
						return finished;
					});
				}
				catch (Exception ignored) {

				}
				finally {
					mActionLock.unlock();
				}
				threadRateControl.doRateControl((int) (m_update_rate * 1000.0));
			}

		});
		mRunnerThread.start();
	}

	public static boolean runAction(AutomatedAction action) {
		return runAction(action, false);
	}

	public static boolean runAction(AutomatedAction action, boolean waitForCompletion) {
		try {
			mActionLock.lock();
			mActionList.removeIf((xAction) -> {
				for (Subsystem xSubsystem : xAction.getRequiredSubsystems()) {
					if (action.getRequiredSubsystems().contains(xSubsystem))
						return true;
				}
				return false;
			});
			mActionList.add(action);
		}
		catch (Exception ignored) {

		}
		finally {
			mActionLock.unlock();
		}

		if (waitForCompletion) {
			boolean completed = false;
			ThreadRateControl threadRateControl = new ThreadRateControl();
			threadRateControl.start();

			TimeoutTimer timeoutTimer = new TimeoutTimer(action.getTimeout());
			while (!timeoutTimer.isTimedOut() && !completed) {
				try {
					mActionLock.lock();
					completed = !mActionList.contains(action);
				} catch (Exception ignored) {

				} finally {
					mActionLock.unlock();
				}
				threadRateControl.doRateControl((int) (m_update_rate * 1000.0));
			}

			if (!completed)
				ConsoleReporter.report(action.getClass().getSimpleName() + " Action did not complete in time!", MessageLevel.ERROR);

			return completed;
		}

		return true;
	}
}
