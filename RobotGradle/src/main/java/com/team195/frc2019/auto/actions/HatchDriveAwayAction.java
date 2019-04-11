package com.team195.frc2019.auto.actions;

import com.team195.frc2019.subsystems.Drive;
import com.team195.frc2019.subsystems.Turret;
import com.team254.lib.util.DriveSignal;
import edu.wpi.first.wpilibj.Timer;

public class HatchDriveAwayAction implements Action {
	private static final Drive mDrive = Drive.getInstance();
	private static final Turret mTurret = Turret.getInstance();

	private double mStartTime;
	private final double mDuration, mLeft, mRight;

	public HatchDriveAwayAction(double duration) {
		mDuration = duration;
		mLeft = 1;
		mRight = 1;
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
		if (!mTurret.getLimitSwitchValue()) {
			mDrive.setOpenLoopAutomated(new DriveSignal(mLeft, mRight));
		}

		mStartTime = Timer.getFPGATimestamp();
	}
}
