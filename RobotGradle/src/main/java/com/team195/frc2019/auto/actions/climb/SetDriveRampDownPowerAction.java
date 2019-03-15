package com.team195.frc2019.auto.actions.climb;

import com.team195.frc2019.auto.actions.Action;
import com.team195.frc2019.subsystems.Drive;
import com.team254.lib.util.DriveSignal;

import java.util.function.Function;

public class SetDriveRampDownPowerAction implements Action {
	private static final Drive mDrive = Drive.getInstance();

	private double mDriveOutput;

	public SetDriveRampDownPowerAction(double driveOutput) {
		mDriveOutput = driveOutput;
	}

	@Override
	public boolean isFinished() {
		return mDriveOutput < 0.05;
	}

	@Override
	public void update() {
		mDriveOutput -= 0.05;
	}

	@Override
	public void done() {
		mDrive.setOpenLoop(new DriveSignal(0, 0));
	}

	@Override
	public void start() {
		mDrive.setClimbLeft(mDriveOutput);
		mDrive.setClimbRight(mDriveOutput);
	}
}
