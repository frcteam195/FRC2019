package com.team195.frc2019.auto.actions;

import com.team195.frc2019.subsystems.VisionTracker;

import java.util.function.Function;

public class SetVisionEnabledHatchSteerAction implements Action {
	private static final VisionTracker mVisionTracker = VisionTracker.getInstance();

	private Function<Void, Boolean> mButtonGetterMethod;

	public SetVisionEnabledHatchSteerAction(Function<Void, Boolean> buttonGetterMethod) {
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
		mVisionTracker.setVisionEnabled(false);
	}

	@Override
	public void start() {
		mVisionTracker.setTargetMode(VisionTracker.TargetMode.HATCH);
		mVisionTracker.setVisionEnabled(true);
	}
}
