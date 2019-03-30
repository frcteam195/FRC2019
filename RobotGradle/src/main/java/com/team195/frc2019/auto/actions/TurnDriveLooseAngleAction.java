package com.team195.frc2019.auto.actions;

import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.subsystems.Drive;
import com.team195.lib.util.TimeoutTimer;
import com.team254.lib.util.DriveSignal;

public class TurnDriveLooseAngleAction implements Action {
	private static final Drive mDrive = Drive.getInstance();

	private final TimeoutTimer mTimeoutTimer;

	private final double mAngle;
	private static final double kTurnThreshold = 25;
	private static final double kTurnkP = 0.014;

	/**
	 * Turn drive loosely to angle
	 * @param angle Positive angle counterclockwise
	 * @param timeout Timeout value in seconds
	 */
	public TurnDriveLooseAngleAction(double angle, double timeout) {
		mAngle = mDrive.getHeading().getDegrees() + angle;
		mTimeoutTimer = new TimeoutTimer(timeout);
	}

	@Override
	public boolean isFinished() {
		ConsoleReporter.report("Gyro Angle: " + mDrive.getHeading().getDegrees());
		return mTimeoutTimer.isTimedOut() || Math.abs(mAngle - mDrive.getHeading().getDegrees()) < kTurnThreshold;
	}

	@Override
	public void update() {
		double steerVal = Math.max(Math.min((mAngle - mDrive.getHeading().getDegrees()) * kTurnkP, 1), -1);
		ConsoleReporter.report("Steer val: " + steerVal);
		mDrive.setOpenLoopAutomated(new DriveSignal(-steerVal, steerVal));
	}

	@Override
	public void done() {
		mDrive.setDriveControlState(Drive.DriveControlState.OPEN_LOOP);
		mDrive.setOpenLoop(new DriveSignal(0, 0));
//		mDrive.setBobbyBrake();
		mDrive.setBrakeMode(false);
	}

	@Override
	public void start() {
		mDrive.setDriveControlState(Drive.DriveControlState.OPEN_LOOP_AUTOMATED);
		double steerVal = Math.max(Math.min((mAngle - mDrive.getHeading().getDegrees()) * kTurnkP, 1), -1);
		mDrive.setBrakeMode(true);
		mDrive.setOpenLoopAutomated(new DriveSignal(-steerVal, steerVal));
	}
}
