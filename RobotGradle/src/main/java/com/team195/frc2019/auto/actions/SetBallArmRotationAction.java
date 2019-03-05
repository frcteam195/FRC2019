package com.team195.frc2019.auto.actions;

import com.team195.frc2019.subsystems.BallIntakeArm;
import com.team195.lib.util.ThreadRateControl;

public class SetBallArmRotationAction implements Action {
	private static final BallIntakeArm mBallArm = BallIntakeArm.getInstance();
	private ThreadRateControl trc = new ThreadRateControl();

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
		mBallArm.setBallIntakeArmControlMode(BallIntakeArm.BallIntakeArmControlMode.DISABLED);
		mBallArm.zeroRemoteSensor();
		trc.start();
		trc.doRateControl(100);
		mBallArm.setBallIntakeArmPosition(mRotation);
		mBallArm.setBallIntakeArmControlMode(BallIntakeArm.BallIntakeArmControlMode.POSITION);
	}
}
