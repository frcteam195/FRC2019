package com.team195.frc2019.auto.actions;

import com.team195.frc2019.subsystems.Turret;
import com.team195.lib.util.TimeoutTimer;

public class BeakOpenAction implements Action {
	private static final Turret mTurret = Turret.getInstance();

	private final TimeoutTimer mTimeoutTimer = new TimeoutTimer(0.1);

	private boolean mOpen;

	public BeakOpenAction(boolean open) {
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
		mTurret.setBeak(mOpen);
	}
}
