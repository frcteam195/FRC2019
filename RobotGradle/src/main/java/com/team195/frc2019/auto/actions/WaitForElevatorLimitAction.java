package com.team195.frc2019.auto.actions;

import com.team195.frc2019.subsystems.Elevator;
import com.team195.lib.util.TimeoutTimer;

public class WaitForElevatorLimitAction implements Action {
	private final Elevator mElevator = Elevator.getInstance();
	private final TimeoutTimer mTimeoutTimer;

	public WaitForElevatorLimitAction (double timeout) {
		mTimeoutTimer = new TimeoutTimer(timeout);
	}

	@Override
	public boolean isFinished() {
		return (mTimeoutTimer.isTimedOut() || mElevator.isAtLowerLimit());
	}

	@Override
	public void update() {
	}

	@Override
	public void done() {
		mElevator.setElevatorPosition(0);
	}

	@Override
	public void start() {
		mTimeoutTimer.reset();
	}
}
