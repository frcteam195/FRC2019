package com.team195.frc2019.auto.actions.climb;

import com.team195.frc2019.auto.actions.Action;
import com.team195.frc2019.subsystems.BallIntakeArm;
import com.team195.frc2019.subsystems.Drive;
import com.team195.lib.util.TimeoutTimer;
import com.team254.lib.util.DriveSignal;

import java.util.function.Function;

public class SetIntakeBarForwardClimbAction implements Action {
	private static final Drive mDrive = Drive.getInstance();
	private static final BallIntakeArm mBallIntakeArm = BallIntakeArm.getInstance();

	private final TimeoutTimer mTimeoutTimer = new TimeoutTimer(10);

	public SetIntakeBarForwardClimbAction() {

	}

	@Override
	public boolean isFinished() {
		return mTimeoutTimer.isTimedOut() || (mDrive.getRoll() > 27);
	}

	@Override
	public void update() {

	}

	@Override
	public void done() {
		mDrive.setOpenLoopRight(0.4);
	}

	@Override
	public void start() {
		mDrive.setOpenLoopRight(0.9);
	}
}
