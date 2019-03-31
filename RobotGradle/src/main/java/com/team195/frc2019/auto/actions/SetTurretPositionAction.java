package com.team195.frc2019.auto.actions;

import com.team195.frc2019.subsystems.Turret;
import com.team195.frc2019.subsystems.positions.TurretPositions;
import com.team195.lib.util.TimeoutTimer;

public class SetTurretPositionAction implements Action {
	private static final Turret mTurret = Turret.getInstance();
	private final TimeoutTimer mTimeoutTimer = new TimeoutTimer(3);

	private double mPosition;

	public SetTurretPositionAction(double position) {
		mPosition = position;
	}

	@Override
	public boolean isFinished() {
		return mTimeoutTimer.isTimedOut() || mTurret.isTurretAtSetpoint(TurretPositions.PositionDelta);
	}

	@Override
	public void update() {
	}

	@Override
	public void done() {

	}

	@Override
	public void start() {
		mTurret.setTurretPosition(mPosition);
	}
}
