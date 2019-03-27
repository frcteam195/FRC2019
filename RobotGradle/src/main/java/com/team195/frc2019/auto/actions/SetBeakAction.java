package com.team195.frc2019.auto.actions;

import com.team195.frc2019.constants.AutoConstants;
import com.team195.frc2019.subsystems.Turret;
import com.team195.lib.util.ThreadRateControl;
import com.team195.lib.util.TimeoutTimer;

public class SetBeakAction implements Action {
	private static final Turret mTurret = Turret.getInstance();
	private ThreadRateControl trc = new ThreadRateControl();
	private final TimeoutTimer mTimeoutTimer = new TimeoutTimer(AutoConstants.kDefaultSolenoidWait * 2.0);

	private boolean mOpen;

	public SetBeakAction(boolean open) {
		mOpen = open;
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
		if (mOpen) {
			mTurret.setBeakFeedOff(false);
			mTurret.setBeak(false);
		}
		else {
			mTurret.setBeakFeedOff(false);
			mTurret.setBeak(true);
			trc.start();
			trc.doRateControl(100);
			mTurret.setBeakFeedOff(true);
		}
	}
}
