package com.team195.frc2019.auto.actions.climb;

import com.team195.frc2019.auto.actions.Action;
import com.team195.frc2019.subsystems.Drive;
import com.team195.lib.util.TimeoutTimer;

import java.util.function.Function;

public class SetClimbRackDownAction implements Action {
	private static final Drive mDrive = Drive.getInstance();

	private final TimeoutTimer mTimeoutTimer = new TimeoutTimer(20);

	private Function<Void, Boolean> mButtonGetterMethod;


	public SetClimbRackDownAction(Function<Void, Boolean> buttonGetterMethod) {
		mButtonGetterMethod = buttonGetterMethod;
	}

	@Override
	public boolean isFinished() {
		return mTimeoutTimer.isTimedOut() || (!mButtonGetterMethod.apply(null));
	}

	@Override
	public void update() {

	}

	@Override
	public void done() {
		mDrive.stop();
	}

	@Override
	public void start() {
		mDrive.setOpenLoopLeft(0.8);
	}
}
