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
		return mBallArm.isArmAtSetpoint(0.05);
	}

	@Override
	public void update() {
	}

	@Override
	public void done() {
		if (mRotation <= 0)
			mBallArm.setBallIntakeArmControlMode(BallIntakeArm.BallIntakeArmControlMode.DISABLED);
	}

	@Override
	public void start() {
		mBallArm.setBallIntakeArmPosition(mRotation);
		mBallArm.setBallIntakeArmControlMode(BallIntakeArm.BallIntakeArmControlMode.POSITION);
	}
}
