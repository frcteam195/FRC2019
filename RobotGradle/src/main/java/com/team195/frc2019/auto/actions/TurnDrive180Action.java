package com.team195.frc2019.auto.actions;

import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.subsystems.Drive;
import com.team195.lib.util.TimeoutTimer;
import com.team254.lib.util.DriveSignal;

public class TurnDrive180Action implements Action {
	private static final Drive mDrive = Drive.getInstance();

	private final TimeoutTimer mTimeoutTimer;

	private final double mAngle;
	private static final double kTurnThreshold = 20;
	private static final double kTurnkP = 0.014;

	/**
	 * Turn drive loosely to angle
	 * @param timeout Timeout value in seconds
	 */
	public TurnDrive180Action(double timeout) {
		if (mDrive.getHeading().getDegrees() > 0)
			mAngle = mDrive.getHeading().getDegrees() - 179.9;
		else
			mAngle = mDrive.getHeading().getDegrees() + 179.9;
		mTimeoutTimer = new TimeoutTimer(timeout);
	}

	@Override
	public boolean isFinished() {
		ConsoleReporter.report("Gyro Angle: " + mDrive.getHeading().getDegrees());
		return mTimeoutTimer.isTimedOut() || Math.abs(mAngle - mDrive.getRawYaw()) < kTurnThreshold;
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
		mDrive.setBrakeMode(true);
	}
}
