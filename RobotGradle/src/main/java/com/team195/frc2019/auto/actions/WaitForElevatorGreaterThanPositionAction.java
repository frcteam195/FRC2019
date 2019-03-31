package com.team195.frc2019.auto.actions;

import com.team195.frc2019.subsystems.Elevator;
import com.team195.frc2019.subsystems.positions.ElevatorPositions;
import com.team195.lib.util.TimeoutTimer;

public class WaitForElevatorGreaterThanPositionAction implements Action {
	private final Elevator mElevator = Elevator.getInstance();
	private final TimeoutTimer mTimeoutTimer;

	private final double mPosition;


	public WaitForElevatorGreaterThanPositionAction(double position, double timeout) {
		mPosition = position;
		mTimeoutTimer = new TimeoutTimer(timeout);
	}

	@Override
	public boolean isFinished() {
		return mTimeoutTimer.isTimedOut() || mElevator.getPosition() > mPosition;
	}

	@Override
	public void update() {
	}

	@Override
	public void done() {

	}

	@Override
	public void start() {

	}
}
