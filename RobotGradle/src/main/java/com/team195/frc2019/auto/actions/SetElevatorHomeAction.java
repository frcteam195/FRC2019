package com.team195.frc2019.auto.actions;

import com.team195.frc2019.subsystems.Elevator;
import com.team195.frc2019.subsystems.Elevator.ElevatorControlMode;

public class SetElevatorHomeAction implements Action {
	private static final Elevator mElevator = Elevator.getInstance();

	public SetElevatorHomeAction() {;}

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
		mElevator.setElevatorPosition(0);
		mElevator.zeroSensors();
		mElevator.setElevatorControlMode(ElevatorControlMode.POSITION);
	}
}
