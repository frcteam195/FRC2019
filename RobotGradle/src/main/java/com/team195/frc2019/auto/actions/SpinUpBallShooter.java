package com.team195.frc2019.auto.actions;

import com.team195.frc2019.subsystems.Turret;
import com.team195.lib.util.TimeoutTimer;

public class SpinUpBallShooter implements Action {
	private static final Turret mTurret = Turret.getInstance();

	private double mOutputSpeed;

	public SpinUpBallShooter(double outputSpeed) {
		mOutputSpeed = outputSpeed;
	}

	@Override
	public boolean isFinished() {
		return mTurret.isShooterAtSetpoint(50);
	}

	@Override
	public void update() {
	}

	@Override
	public void done() {

	}

	@Override
	public void start() {
		mTurret.setBallShooterVelocity(mOutputSpeed);
	}
}
