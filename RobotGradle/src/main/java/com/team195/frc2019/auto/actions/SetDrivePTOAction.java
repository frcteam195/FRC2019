package com.team195.frc2019.auto.actions;

import com.team195.frc2019.constants.AutoConstants;
import com.team195.frc2019.subsystems.Drive;
import com.team195.lib.util.TimeoutTimer;
import com.team254.lib.util.DriveSignal;

public class SetDrivePTOAction implements Action {
	private static final Drive mDrive = Drive.getInstance();

	private final TimeoutTimer mTimeoutTimer = new TimeoutTimer(AutoConstants.kDefaultSolenoidWait);

	private boolean mDriveClimber;

	public SetDrivePTOAction(boolean driveClimber) {
		mDriveClimber = driveClimber;
	}

	@Override
	public boolean isFinished() {
		return mTimeoutTimer.isTimedOut();
	}

	@Override
	public void update() {
	}

	@Override
	public void done() {

	}

	@Override
	public void start() {
		if (mDriveClimber) {
			mDrive.setDriveControlState(Drive.DriveControlState.CLIMB);
		} else {
			mDrive.setOpenLoop(new DriveSignal(0, 0));
		}
		
		mDrive.setPTO(mDriveClimber);
		mTimeoutTimer.reset();
	}
}
