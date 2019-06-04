package com.team195.frc2019.auto.actions;

import com.team195.frc2019.subsystems.Turret;
import com.team195.lib.util.TimeoutTimer;

public class WaitForTurretLessThanRotationAction implements Action {
	private final Turret mTurret = Turret.getInstance();
	private final TimeoutTimer mTimeoutTimer;

	private final double mPosition;


	public WaitForTurretLessThanRotationAction(double position, double timeout) {
		mPosition = position;
		mTimeoutTimer = new TimeoutTimer(timeout);
	}

	@Override
	public boolean isFinished() {
		return mTimeoutTimer.isTimedOut() || Math.abs(mTurret.getPosition()) < mPosition;
	}

	@Override
	public void update() {
	}

	@Override
	public void done() {

	}

	@Override
	public void start() {
		mTimeoutTimer.reset();
	}
}
