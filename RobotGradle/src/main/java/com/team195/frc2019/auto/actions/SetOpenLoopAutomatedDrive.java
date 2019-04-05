package com.team195.frc2019.auto.actions;

import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.subsystems.Drive;
import com.team254.lib.util.DriveSignal;
import edu.wpi.first.wpilibj.Timer;

public class SetOpenLoopAutomatedDrive implements Action {
	private static final Drive mDrive = Drive.getInstance();

	private double mStartTime;
	private final double mDuration, mLeft, mRight;

	public SetOpenLoopAutomatedDrive(double left, double right, double duration) {
		mDuration = duration;
		mLeft = left;
		mRight = right;
	}

	@Override
	public boolean isFinished() {
		return Timer.getFPGATimestamp() - mStartTime > mDuration;
	}

	@Override
	public void update() {
//		ConsoleReporter.report((Timer.getFPGATimestamp() - mStartTime) + " > " + mDuration);

	}

	@Override
	public void done() {
		mDrive.setOpenLoop(new DriveSignal(0.0, 0.0));
	}

	@Override
	public void start() {
		mDrive.setOpenLoopAutomated(new DriveSignal(mLeft, mRight));
		mStartTime = Timer.getFPGATimestamp();
	}
}
