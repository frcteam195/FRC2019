package com.team195.frc2019.auto.actions;

import com.team195.frc2019.constants.AutoConstants;
import com.team195.frc2019.subsystems.Turret;
import com.team195.lib.util.TimeoutTimer;

public class SetHatchPushAction implements Action {
	private static final Turret mTurret = Turret.getInstance();

	private final TimeoutTimer mTimeoutTimer = new TimeoutTimer(AutoConstants.kDefaultSolenoidWait);

	private boolean mPushOut;

	public SetHatchPushAction(boolean pushOut) {
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
