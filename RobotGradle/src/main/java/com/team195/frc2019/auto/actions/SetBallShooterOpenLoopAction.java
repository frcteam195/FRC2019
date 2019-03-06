package com.team195.frc2019.auto.actions;

import com.team195.frc2019.auto.AutoConstants;
import com.team195.frc2019.subsystems.Turret;
import com.team195.lib.util.TimeoutTimer;

public class SetBallShooterOpenLoopAction implements Action {
	private static final Turret mTurret = Turret.getInstance();
	private final TimeoutTimer mTimeoutTimer = new TimeoutTimer(AutoConstants.kDefaultRollerSpinUpWait);

	private double mOutputSpeed;

	public SetBallShooterOpenLoopAction(double outputSpeed) {
		mOutputSpeed = outputSpeed;
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
		mTurret.setBallShooterOpenLoop(mOutputSpeed);
	}
}
