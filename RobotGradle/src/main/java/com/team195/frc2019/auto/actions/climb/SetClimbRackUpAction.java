package com.team195.frc2019.auto.actions.climb;

import com.team195.frc2019.auto.actions.Action;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.subsystems.Drive;
import com.team195.lib.util.TimeoutTimer;


public class SetClimbRackUpAction implements Action {
	private static final Drive mDrive = Drive.getInstance();

	private final TimeoutTimer mTimeoutTimer = new TimeoutTimer(3);


	public SetClimbRackUpAction() {

	}

	@Override
	public boolean isFinished() {
		return mTimeoutTimer.isTimedOut() || Math.abs(mDrive.getRawLeftSparkEncoder()) < 3;
	}

	@Override
	public void update() {

	}

	@Override
	public void done() {
		mDrive.setClimbLeft(0);
		mDrive.setClimbRight(0);
	}

	@Override
	public void start() {
		mDrive.setClimbLeft(-0.4);
	}
}
