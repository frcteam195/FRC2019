package com.team195.frc2019.auto.actions;

import com.team195.frc2019.subsystems.Drive;
import com.team254.lib.util.DriveSignal;

import java.util.function.Function;

public class SetOpenLoopDriveAction implements Action {
	private static final Drive mDrive = Drive.getInstance();

	Function<Void, Boolean> mButtonGetterMethod;
	Function<Void, Double> mAxisLeftGetterMethod;
	Function<Void, Double> mAxisRightGetterMethod;

	public SetOpenLoopDriveAction(Function<Void, Boolean> buttonGetterMethod, Function<Void, Double> axisLeftGetterMethod, Function<Void, Double> axisRightGetterMethod) {
		mButtonGetterMethod = buttonGetterMethod;
		mAxisLeftGetterMethod = axisLeftGetterMethod;
		mAxisRightGetterMethod = axisRightGetterMethod;
	}

	@Override
	public boolean isFinished() {
		return (!mButtonGetterMethod.apply(null));
	}

	@Override
	public void update() {
		mDrive.setClimbLeft(mAxisLeftGetterMethod.apply(null));
		mDrive.setClimbRight(mAxisRightGetterMethod.apply(null));
	}

	@Override
	public void done() {
		mDrive.setOpenLoop(new DriveSignal(0, 0));
	}

	@Override
	public void start() {
		mDrive.setClimbLeft(0);
		mDrive.setClimbRight(0);
	}
}
