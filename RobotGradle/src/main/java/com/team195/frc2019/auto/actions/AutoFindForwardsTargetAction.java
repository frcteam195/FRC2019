package com.team195.frc2019.auto.actions;

import com.team195.frc2019.subsystems.Drive;
import com.team195.frc2019.subsystems.Turret;
import com.team195.frc2019.subsystems.VisionTracker;
import com.team195.frc2019.subsystems.positions.TurretPositions;
import com.team195.lib.util.TimeoutTimer;
import com.team254.lib.util.CheesyDriveHelper;
import com.team254.lib.util.DriveSignal;

public class AutoFindForwardsTargetAction implements Action {
	private static final VisionTracker mVisionTracker = VisionTracker.getInstance();
	private static final Turret mTurret = Turret.getInstance();
	private static final Drive mDrive = Drive.getInstance();
	private CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();
	private TimeoutTimer mTimeoutTimer = new TimeoutTimer(4);

	private final boolean mReverse;

	public AutoFindForwardsTargetAction(boolean direction) {
		mReverse = direction;
	}

	@Override
	public boolean isFinished() {
		return mTimeoutTimer.isTimedOut() || mVisionTracker.isTargetAreaReached();
	}

	@Override
	public void update() {
		double throttle = mReverse ? -0.25 : 0.25;
		double turn = 0;
		if (VisionTracker.getInstance().isTargetFound()) {
			turn = Math.max(Math.min(VisionTracker.getInstance().getTargetHorizAngleDev() * 0.01, 0.1), -0.1);
		}
		mDrive.setOpenLoopAutomated(mCheesyDriveHelper.cheesyDrive(throttle, turn, true, true));
	}

	@Override
	public void done() {
		mDrive.setOpenLoopAutomated(DriveSignal.NEUTRAL);
		mVisionTracker.setVisionEnabled(false);
	}

	@Override
	public void start() {
		mDrive.setBrakeMode(true);
		mDrive.forceBrakeModeUpdate();
		mVisionTracker.setTargetMode(VisionTracker.TargetMode.HATCH);
		mVisionTracker.setVisionEnabled(true);
	}
}
