package com.team195.frc2019.auto.actions;

import com.team195.frc2019.subsystems.BallIntakeArm;

public class SetBallArmRotationAction implements Action {
	private static final BallIntakeArm mBallArm = BallIntakeArm.getInstance();

	private double mRotation;

	public SetBallArmRotationAction(double armRotation) {
		mRotation = armRotation;
	}

	@Override
	public boolean isFinished() {
		return mBallArm.isElevatorAtSetpoint(0.2);
	}

	@Override
	public void update() {
	}

	@Override
	public void done() {

	}

	@Override
	public void start() {
		mBallArm.setBallIntakeArmPosition(0);
		mBallArm.setBallIntakeArmControlMode(BallIntakeArm.BallIntakeArmControlMode.POSITION);
		mBallArm.setBallIntakeArmPosition(mRotation);
	}
}
