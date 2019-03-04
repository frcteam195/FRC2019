package com.team195.frc2019.auto.actions;

import com.team195.frc2019.subsystems.Elevator;

public class SetElevatorHeightAction  implements Action {
	private static final Elevator mElevator = Elevator.getInstance();

	private double mHeight;

	public SetElevatorHeightAction(double elevatorHeight) {
		mHeight = elevatorHeight;
	}

	@Override
	public boolean isFinished() {
		return mElevator.isElevatorAtSetpoint(0.2);
	}

	@Override
	public void update() {
	}

	@Override
	public void done() {

	}

	@Override
	public void start() {
		mElevator.setElevatorPosition(mHeight);
	}
}
