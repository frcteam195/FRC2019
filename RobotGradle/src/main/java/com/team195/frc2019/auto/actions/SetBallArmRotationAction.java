package com.team195.frc2019.auto.actions;

import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.subsystems.BallIntakeArm;
import com.team195.frc2019.subsystems.positions.BallIntakeArmPositions;
import com.team195.lib.util.ThreadRateControl;
import com.team195.lib.util.TimeoutTimer;

import java.sql.Time;

public class SetBallArmRotationAction implements Action {
	private static final BallIntakeArm mBallArm = BallIntakeArm.getInstance();
	private final TimeoutTimer mTimeoutTimer;

	private final TimeoutTimer mEncoderResetWait = new TimeoutTimer(0.1);
	private ArmActionState mArmActionState = ArmActionState.WAITING;

	private double mRotation;

	public SetBallArmRotationAction(double armRotation) {
		this(armRotation, 3);
	}

	public SetBallArmRotationAction(double armRotation, double timeout) {
		mRotation = armRotation;
		mTimeoutTimer = new TimeoutTimer(timeout);
	}

	@Override
	public boolean isFinished() {
//		ConsoleReporter.report("Arm Encoder Pos: " + mBallArm.getPosition());
		return mTimeoutTimer.isTimedOut()
				|| (mBallArm.getSetpoint() == BallIntakeArmPositions.Down  && mBallArm.isArmAtSetpoint(0.2))
				|| (mBallArm.getSetpoint() == BallIntakeArmPositions.Up && mBallArm.isArmUp());
	}

	@Override
	public void update() {
		switch (mArmActionState) {
			case WAITING:
				if (mEncoderResetWait.isTimedOut())
					mArmActionState = ArmActionState.RUNNING;
				break;
			case RUNNING:
				mBallArm.setBallIntakeArmPosition(mRotation);
				if (mRotation > 0) {
					mBallArm.setBallIntakeRollerSpeed(BallIntakeArmPositions.RollerOff);
					mBallArm.setBallIntakeArmControlMode(BallIntakeArm.BallIntakeArmControlMode.OPEN_LOOP);
				}
				else {
					mBallArm.setBallIntakeArmControlMode(BallIntakeArm.BallIntakeArmControlMode.POSITION);
				}
				mArmActionState = ArmActionState.DONE;
				break;
			case DONE:
			default:
				break;
		}

	}

	@Override
	public void done() {
		if (mRotation <= 0)
			mBallArm.setBallIntakeArmControlMode(BallIntakeArm.BallIntakeArmControlMode.DISABLED);
	}

	@Override
	public void start() {
		if (mBallArm.getSetpoint() != mRotation) {
			//If the arm is up, zero sensor
			if (mRotation < 0) {
				mBallArm.setBallIntakeArmControlMode(BallIntakeArm.BallIntakeArmControlMode.DISABLED);
				mBallArm.zeroSensors();
				mBallArm.setBallIntakeArmPosition(0);
				mArmActionState = ArmActionState.WAITING;
			}
			else {
				mArmActionState = ArmActionState.RUNNING;
			}
		}
	}

	private enum ArmActionState {
		WAITING,
		RUNNING,
		DONE;
	}
}
