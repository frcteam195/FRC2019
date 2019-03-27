package com.team195.lib.util;

import com.team195.frc2019.auto.autonomy.AutomatedAction;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.reporters.MessageLevel;
import com.team195.frc2019.subsystems.Subsystem;

import java.util.*;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.ReentrantLock;

public class TeleopActionRunner {
	private static double m_update_rate = 1.0 / 50.0;    //20ms update rate
	private static HashSet<AutomatedAction> mActionList = new HashSet<>();
	private static ReentrantLock mActionLock = new ReentrantLock();

	static {
		Thread mRunnerThread = new Thread(() -> {
			Thread.currentThread().setName("TeleopActionRunner");
			ThreadRateControl threadRateControl = new ThreadRateControl();
			threadRateControl.start();

			while (true) {
				try {
					ConsoleReporter.report("Acquiring Lock", MessageLevel.INFO);
					if (mActionLock.tryLock()) {
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
						} finally {
							ConsoleReporter.report("Releasing Lock", MessageLevel.INFO);
							mActionLock.unlock();
						}
						threadRateControl.doRateControl((int) (m_update_rate * 1000.0));
					}
				}
				catch (Exception ex) {
					ConsoleReporter.report(ex);
				}
			}

		});
		mRunnerThread.start();
	}

	public synchronized static void init() {
		;
	}

	public static boolean runAction(AutomatedAction action) {
		return runAction(action, false);
	}

	public static boolean runAction(AutomatedAction action, boolean waitForCompletion) {
		try {
			ConsoleReporter.report("Acquiring Lock", MessageLevel.INFO);
			if (mActionLock.tryLock(50, TimeUnit.MILLISECONDS)) {
				try {
					if (mActionList.size() > 0) {
						mActionList.removeIf((xAction) -> {
							for (Subsystem xSubsystem : xAction.getRequiredSubsystems()) {
								if (action.getRequiredSubsystems().contains(xSubsystem))
									return true;
							}
							return false;
						});
					}
					mActionList.add(action);
				} catch (Exception ex) {
					ConsoleReporter.report(ex);
				} finally {
					ConsoleReporter.report("Releasing Lock", MessageLevel.INFO);
					mActionLock.unlock();
				}
			}
		}
		catch (Exception ex) {
			ConsoleReporter.report(ex);
		}

		if (waitForCompletion) {
			boolean completed = false;
			ThreadRateControl threadRateControl = new ThreadRateControl();
			threadRateControl.start();

			TimeoutTimer timeoutTimer = new TimeoutTimer(action.getTimeout());
			while (!timeoutTimer.isTimedOut() && !completed) {
				ConsoleReporter.report("Acquiring Lock", MessageLevel.INFO);
				if (mActionLock.tryLock()) {
					try {
						completed = !mActionList.contains(action);
					} catch (Exception ex) {
						ConsoleReporter.report(ex);
					} finally {
						ConsoleReporter.report("Releasing Lock", MessageLevel.INFO);
						mActionLock.unlock();
					}
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
