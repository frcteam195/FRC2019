package com.team195.frc2019.auto.actions;

import com.team195.frc2019.subsystems.Turret;
import com.team195.lib.util.TimeoutTimer;

public class HatchPushAction implements Action {
	private static final Turret mTurret = Turret.getInstance();

	private final TimeoutTimer mTimeoutTimer = new TimeoutTimer(0.1);

	private boolean mPushOut;

	public HatchPushAction(boolean pushOut) {
		mPushOut = pushOut;
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
		mTurret.setHatchPush(mPushOut);
	}
}
