package com.team195.lib.util;

public class TimeoutTimer {
	private double timeout;
	private ElapsedTimer eTimer = new ElapsedTimer();
	private boolean firstRun;

	public TimeoutTimer(double timeout) {
		this.timeout = timeout;
		setFirstRun(true);
	}

	public boolean isTimedOut() {
		if (firstRun) {
			eTimer.start();
			setFirstRun(false);
		}
		return eTimer.hasElapsed() > timeout;
	}

	public double getTimeLeft() {
		return Math.max(timeout - eTimer.hasElapsed(), 0);
	}

	public void reset() {
		setFirstRun(true);
	}

	private synchronized void setFirstRun(boolean firstRun) {
		this.firstRun = firstRun;
	}
}
