package com.team195.frc2019.auto.actions.climb;

import com.team195.frc2019.auto.actions.Action;
import com.team195.frc2019.subsystems.Drive;
import com.team254.lib.util.DriveSignal;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;
import java.util.function.Predicate;

public class HoldOpenLoopDriveClimbAction implements Action {
	private static final Drive mDrive = Drive.getInstance();

	private final double mLeft, mRight;

	private final AtomicReference<Predicate<Void>> mButtonGetterMethod;


	public HoldOpenLoopDriveClimbAction(double left, double right, AtomicReference<Predicate<Void>> buttonGetterMethod) {
		mLeft = left;
		mRight = right;
		mButtonGetterMethod = buttonGetterMethod;
	}

	@Override
	public boolean isFinished() {
		if (mButtonGetterMethod.get() != null)
			return (!mButtonGetterMethod.get().test(null));
		else
			return true;
	}

	@Override
	public void update() {

	}

	@Override
	public void done() {
		mDrive.setOpenLoop(new DriveSignal(0.0, 0.0));
	}

	@Override
	public void start() {
		mDrive.configureClimbCurrentLimit();
		mDrive.setOpenLoopAutomated(new DriveSignal(mLeft, mRight));
	}
}
