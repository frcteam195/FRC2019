package com.team195.frc2019.auto.actions.climb;

import com.team195.frc2019.auto.actions.Action;
import com.team195.frc2019.subsystems.Drive;
import com.team254.lib.util.DriveSignal;

import java.util.function.Function;

public class HoldOpenLoopDriveClimbAction implements Action {
	private static final Drive mDrive = Drive.getInstance();

	private final double mLeft, mRight;

	private final Function<Void, Boolean> mButtonGetterMethod;


	public HoldOpenLoopDriveClimbAction(double left, double right, Function<Void, Boolean> buttonGetterMethod) {
		mLeft = left;
		mRight = right;
		mButtonGetterMethod = buttonGetterMethod;
	}

	@Override
	public boolean isFinished() {
		return (!mButtonGetterMethod.apply(null));
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
