package com.team195.lib.util;

import com.team195.frc2019.Constants;
import com.team195.frc2019.auto.actions.Action;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.reporters.MessageLevel;

public class TeleopActionRunner {
	private double m_update_rate = 1.0 / 50.0;    //20ms update rate
	private Action action;
	private boolean finished = false;
	private TimeoutTimer timeoutTimer;
	private Thread t;
	//TODO: Test the new timeout works properly

	/**
	 * Create an action runner with a timeout
	 * @param action Action to run
	 * @param timeout Timeout in seconds.
	 */
	public TeleopActionRunner(Action action, double timeout) {
		this.action = action;
		timeoutTimer = new TimeoutTimer(timeout);
	}

	public TeleopActionRunner(Action action) {
		this(action, Constants.kActionTimeoutS);
	}

	public boolean isFinished() {
		return finished;
	}

	private void start() {
		timeoutTimer.isTimedOut();
		t = new Thread(() -> {
			//t.setPriority(Thread.NORM_PRIORITY);
			ThreadRateControl threadRateControl = new ThreadRateControl();
			threadRateControl.start();
			action.start();

			while (!action.isFinished() && !timeoutTimer.isTimedOut()) {
				action.update();
				threadRateControl.doRateControl((int)(m_update_rate * 1000.0));
			}

			action.done();
			finished = true;
		});
		t.start();
	}

	public boolean runAction() {
		return runAction(false);
	}

	public boolean runAction(boolean waitForCompletion) {
		if (t == null || !t.isAlive())
			start();

		if (waitForCompletion && timeoutTimer.getTimeLeft() > 0) {
			try {
				t.join((int) (timeoutTimer.getTimeLeft() * 1000.0));
			} catch (InterruptedException ex) {
				ConsoleReporter.report(action.getClass().getSimpleName() + " Action did not complete in time!", MessageLevel.ERROR);
				ConsoleReporter.report(ex, MessageLevel.ERROR);
			}
		}

		return finished;
	}
}
