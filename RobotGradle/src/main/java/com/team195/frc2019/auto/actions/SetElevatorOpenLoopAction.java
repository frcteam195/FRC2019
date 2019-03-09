package com.team195.frc2019.auto.actions;

import com.team195.frc2019.subsystems.Elevator;
import com.team195.frc2019.subsystems.Elevator.ElevatorControlMode;

public class SetElevatorOpenLoopAction implements Action {
	private static final Elevator mElevator = Elevator.getInstance();

	private double mSpeed;

	public SetElevatorOpenLoopAction(double speed) {
		mSpeed = speed;
	}

	@Override
	public boolean isFinished() {
		// return mElevator.isElevatorAtSetpoint(0.2);
		return true;
	}

	@Override
	public void update() {
	}

	@Override
	public void done() {

	}

	@Override
	public void start() {
		mElevator.setElevatorControlMode(ElevatorControlMode.OPEN_LOOP);
		mElevator.setElevatorPosition(0);
	}
}
