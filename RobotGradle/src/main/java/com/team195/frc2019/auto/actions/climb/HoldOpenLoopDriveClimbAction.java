package com.team195.frc2019.auto.actions.climb;

import com.team195.frc2019.auto.actions.Action;
import com.team195.frc2019.subsystems.Drive;
import com.team195.lib.util.TimeoutTimer;
import com.team254.lib.util.DriveSignal;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;
import java.util.function.Predicate;

public class HoldOpenLoopDriveClimbAction implements Action {
	private static final Drive mDrive = Drive.getInstance();

	private final double mLeft, mRight;

	private final AtomicReference<Predicate<Void>> mButtonGetterMethod;

	private final TimeoutTimer mTimeoutTimer = new TimeoutTimer(0.1);


	public HoldOpenLoopDriveClimbAction(double left, double right, AtomicReference<Predicate<Void>> buttonGetterMethod) {
		mLeft = left;
		mRight = right;
		mButtonGetterMethod = buttonGetterMethod;
	}

	@Override
	public boolean isFinished() {
		if (mButtonGetterMethod.get() != null)
			return (!mButtonGetterMethod.get().test(null)) && mTimeoutTimer.isTimedOut();
		else
			return true && mTimeoutTimer.isTimedOut();
	}

	@Override
	public void update() {
		if (mTimeoutTimer.isTimedOut())
			mDrive.setOpenLoopAutomated(new DriveSignal(mLeft, mRight));
	}

	@Override
	public void done() {
		mDrive.setOpenLoop(new DriveSignal(0.0, 0.0));
	}

	@Override
	public void start() {
		mDrive.configureClimbCurrentLimit();
		mTimeoutTimer.reset();
		mDrive.setOpenLoopAutomated(new DriveSignal(0, mRight));
	}
}
