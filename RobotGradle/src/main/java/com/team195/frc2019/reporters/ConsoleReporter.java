package com.team195.frc2019.reporters;

import com.team195.frc2019.constants.Constants;
import com.team195.lib.util.ThreadRateControl;
import com.team254.lib.util.CrashTrackingRunnable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;

import java.io.FileWriter;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.Date;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.ReentrantLock;

/**
 * A class to report messages to the console or DriverStation. Messages with a level of DEFCON1 will always be reported
 * whether reporting is enabled or not and will be reported both to the console and the DriverStation.
 */
public class ConsoleReporter {

	private static final double MIN_CONSOLE_SEND_RATE_MS = 0.500;
	private static MessageLevel reportingLevel = MessageLevel.ERROR;
	private static LinkedHashSet<CKMessage> sendMessageSet = new LinkedHashSet<>();
	private static ReentrantLock _reporterMutex = new ReentrantLock();
	private static ConsoleReporter instance = null;

	private boolean firstRun = true;
	private final Notifier mConsoleNotifier;
	private final Object taskRunningLock_ = new Object();

	private ConsoleReporter() {
		mConsoleNotifier = new Notifier(mConsoleRunnable);
		mConsoleNotifier.startPeriodic(MIN_CONSOLE_SEND_RATE_MS);

	}

	public static ConsoleReporter getInstance() {
		if(instance == null) {
			try {
				instance = new ConsoleReporter();
			} catch (Exception ex) {
				ex.printStackTrace();
			}
		}

		return instance;
	}

	public static void setReportingLevel(MessageLevel messageLevel) {
		ConsoleReporter.reportingLevel = messageLevel;
	}

	public static void report(Throwable t) { report(t, MessageLevel.ERROR); }

	public static void report(Throwable t, MessageLevel messageLevel) {
		StringWriter s = new StringWriter();
		t.printStackTrace(new PrintWriter(s));
		report(s.toString(), messageLevel);
	}

	public static void report(Object message) {
		report(String.valueOf(message));
	}

	public static void report(Object message, MessageLevel msgLvl) {
		report(String.valueOf(message), msgLvl);
	}

	public static void report(String message) {
		report(message, MessageLevel.WARNING);
	}

	public static void report(String message, MessageLevel msgLvl) {
		if (msgLvl == MessageLevel.DEFCON1 || (Constants.REPORTING_ENABLED && (msgLvl.ordinal() <= reportingLevel.ordinal()))) {
			try {
				if (_reporterMutex.tryLock(10, TimeUnit.MILLISECONDS)) {
					try {
						sendMessageSet.add(new CKMessage(message, msgLvl));
					} finally {
						_reporterMutex.unlock();
					}
				}
			} catch (Exception ignored) {

			}
		}
	}

	private final CrashTrackingRunnable mConsoleRunnable = new CrashTrackingRunnable() {
		@Override
		public void runCrashTracked() {
			if (firstRun) {
				Thread.currentThread().setName("ConsoleReporter");
				Thread.currentThread().setPriority(Constants.kConsoleReporterThreadPriority);
				firstRun = false;
			}
			try {
				if(_reporterMutex.tryLock(100, TimeUnit.MILLISECONDS)) {
					try {
						for (Iterator<CKMessage> i = sendMessageSet.iterator(); i.hasNext(); ) {
							CKMessage ckm = i.next();
							if (ckm.messageLevel == MessageLevel.DEFCON1 || (Constants.REPORTING_ENABLED && (ckm.messageLevel.ordinal() <= reportingLevel.ordinal()))) {
								String s = ckm.toString();
								if (Constants.REPORT_TO_DRIVERSTATION_INSTEAD_OF_CONSOLE) {
									switch (ckm.messageLevel) {
										case DEFCON1:
											System.out.println(s);
										case ERROR:
											DriverStation.reportError(s, false);
											break;
										case WARNING:
										case INFO:
											DriverStation.reportWarning(s, false);
											break;
										default:
											break;
									}
								} else {
									System.out.println(s);
									if (ckm.messageLevel == MessageLevel.DEFCON1)
										DriverStation.reportError(s, false);
								}

								i.remove();
							}
						}
					} finally {
						_reporterMutex.unlock();
					}
				}
			} catch (Exception ex) {
				ex.printStackTrace();
			}
		}
	};

}
