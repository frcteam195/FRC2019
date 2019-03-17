package com.team195.frc2019.auto.actions;

import com.team195.frc2019.auto.AutoConstants;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.subsystems.BallIntakeArm;
import com.team195.frc2019.subsystems.positions.BallIntakeArmPositions;
import com.team195.lib.util.ThreadRateControl;
import com.team195.lib.util.TimeoutTimer;

public class SetBallArmRotationAction implements Action {
	private static final BallIntakeArm mBallArm = BallIntakeArm.getInstance();
	private final TimeoutTimer mTimeoutTimer = new TimeoutTimer(2);

	private ThreadRateControl trc = new ThreadRateControl();

	private double mRotation;

	public SetBallArmRotationAction(double armRotation) {
		mRotation = armRotation;
	}

	@Override
	public boolean isFinished() {
		ConsoleReporter.report("Arm Encoder Pos: " + mBallArm.getPosition());
		return mTimeoutTimer.isTimedOut() || mBallArm.isArmAtSetpoint(0.5);
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
		if (mBallArm.getSetpoint() != mRotation) {
			if (mRotation > 0)
				mBallArm.setBallIntakeRollerSpeed(BallIntakeArmPositions.RollerOff);
			mBallArm.setBallIntakeArmControlMode(BallIntakeArm.BallIntakeArmControlMode.DISABLED);
			mBallArm.zeroRemoteSensor();
			trc.start();
			trc.doRateControl(100);
			mBallArm.setBallIntakeArmPosition(mRotation);
			mBallArm.setBallIntakeArmControlMode(BallIntakeArm.BallIntakeArmControlMode.POSITION);
		}
	}
}
