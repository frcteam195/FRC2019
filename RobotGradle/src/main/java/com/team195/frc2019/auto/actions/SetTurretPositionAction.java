package com.team195.frc2019.auto.actions;

import com.team195.frc2019.subsystems.Turret;
import com.team195.frc2019.subsystems.positions.TurretPositions;

public class SetTurretPositionAction implements Action {
	private static final Turret mTurret = Turret.getInstance();

	private double mPosition;

	public SetTurretPositionAction(double position) {
		mPosition = position;
	}

	@Override
	public boolean isFinished() {
		return mTurret.isTurretAtSetpoint(TurretPositions.PositionDelta);
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
