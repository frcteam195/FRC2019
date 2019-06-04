package com.team195.frc2019.auto.actions;

import com.team195.frc2019.constants.AutoConstants;
import com.team195.frc2019.subsystems.Turret;
import com.team195.lib.util.ThreadRateControl;
import com.team195.lib.util.TimeoutTimer;

public class SetBeakAction implements Action {
	private static final Turret mTurret = Turret.getInstance();
	private final TimeoutTimer mTimeoutTimer = new TimeoutTimer(AutoConstants.kDefaultSolenoidWait * 2.0);

	private final TimeoutTimer mFeedOffTimeout = new TimeoutTimer(0.2);
	private BeakActionState mBeakActionState = BeakActionState.FEED_ON;

	private boolean mOpen;

	public SetBeakAction(boolean open) {
		mOpen = open;
	}

	@Override
	public boolean isFinished() {
		return mTimeoutTimer.isTimedOut() && mBeakActionState == BeakActionState.DONE;
	}

	@Override
	public void update() {
		if (mBeakActionState == BeakActionState.FEED_OFF) {
			if (mFeedOffTimeout.isTimedOut()) {
				mTurret.setBeakFeedOff(true);
				mBeakActionState = BeakActionState.DONE;
			}
		} else {
			mBeakActionState = BeakActionState.DONE;
		}
	}

	@Override
	public void done() {

	}

	@Override
	public void start() {
		mTimeoutTimer.reset();
		mFeedOffTimeout.reset();

		if (mOpen) {
			mTurret.setBeakFeedOff(false);
			mTurret.setBeak(false);
			mBeakActionState = BeakActionState.FEED_ON;
		}
		else {
			mTurret.setBeakFeedOff(false);
			mTurret.setBeak(true);
			mBeakActionState = BeakActionState.FEED_OFF;
		}
	}

	private enum BeakActionState {
		FEED_OFF,
		FEED_ON,
		DONE;
	}
}
