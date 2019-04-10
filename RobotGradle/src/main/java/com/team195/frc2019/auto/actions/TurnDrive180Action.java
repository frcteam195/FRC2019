package com.team195.frc2019.auto.actions;

import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.subsystems.Drive;
import com.team195.lib.util.TimeoutTimer;
import com.team254.lib.util.DriveSignal;

public class TurnDrive180Action implements Action {
	private static final Drive mDrive = Drive.getInstance();

	private final TimeoutTimer mTimeoutTimer;

	private final double mAngle;
	private static final double kTurnThreshold = 35;
	private static final double kTurnkP = 0.014;

	/**
	 * Turn drive loosely to angle
	 * @param timeout Timeout value in seconds
	 */
	public TurnDrive180Action(double timeout) {
		double startingAngle = convertTo2PIRange(mDrive.getHeading().getDegrees());
		mAngle = convertTo2PIRange(startingAngle + 180);
		ConsoleReporter.report("Starting turn - startingAngle" + startingAngle + " mAngle: " + mAngle);
		mTimeoutTimer = new TimeoutTimer(timeout);
	}

	@Override
	public boolean isFinished() {
		ConsoleReporter.report("Gyro Angle: " + mDrive.getHeading().getDegrees());
		double err = convertTo2PIRange(mAngle - convertTo2PIRange(mDrive.getHeading().getDegrees()));
		return mTimeoutTimer.isTimedOut() || err < kTurnThreshold || err > 360 - kTurnThreshold;
	}

	@Override
	public void update() {
		double steerVal = Math.max(Math.min((convertTo2PIRange(mAngle - convertTo2PIRange(mDrive.getHeading().getDegrees()))) * kTurnkP, 1), -1);
		ConsoleReporter.report("Actual: " + convertTo2PIRange(mDrive.getHeading().getDegrees()) + "Steer val: " + steerVal);
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

	private static double convertTo2PIRange(double angle) {
		while (angle < 0)
			angle += 360;
		while (angle > 360)
			angle -= 360;
		return angle;
	}
}
