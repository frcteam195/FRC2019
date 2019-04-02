package com.team195.frc2019.auto.actions;

import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.subsystems.Drive;
import com.team254.lib.util.DriveSignal;

public class SetDriveBaseVelocity implements Action {

	double mLeftOutput;
	double mRightOutput;
	boolean mBrakeMode;

	public SetDriveBaseVelocity(double leftOutput, double rightOutput) {
		this(leftOutput, rightOutput, true);
	}

	public SetDriveBaseVelocity(double leftOutput, double rightOutput, boolean brakeMode) {
		mLeftOutput = leftOutput;
		mRightOutput = rightOutput;
		mBrakeMode = brakeMode;
	}

	public String toString() {
		return getClass().getSimpleName() + ": LV:" + mLeftOutput + " RV:" + mRightOutput;
	}

	@Override
	public boolean isFinished() {
		return true;
	}

	@Override
	public void update() {
		ConsoleReporter.report(toString());
	}

	@Override
	public void done() {

	}

	@Override
	public void start() {
		Drive.getInstance().setVelocity(new DriveSignal(mLeftOutput, mRightOutput), new DriveSignal(0, 0));
	}
}
