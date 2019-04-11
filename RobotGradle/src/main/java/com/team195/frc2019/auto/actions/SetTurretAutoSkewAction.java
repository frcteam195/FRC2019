package com.team195.frc2019.auto.actions;

import com.team195.frc2019.subsystems.Turret;
import com.team195.frc2019.subsystems.VisionTracker;
import com.team195.frc2019.subsystems.positions.TurretPositions;
import com.team195.lib.util.TimeoutTimer;

public class SetTurretAutoSkewAction implements Action {
	private static final Turret mTurret = Turret.getInstance();
	private static final VisionTracker mVisionTracker = VisionTracker.getInstance();
	private final TimeoutTimer mTimeoutTimer = new TimeoutTimer(3);

	private static final double kTurretSkewFactor = 0.5;

	public SetTurretAutoSkewAction() {

	}

	@Override
	public boolean isFinished() {
		return mTimeoutTimer.isTimedOut()
				|| mTurret.isTurretAtSetpoint(TurretPositions.PositionDelta)
				|| !mVisionTracker.isVisionEnabled();
	}

	@Override
	public void update() {
	}

	@Override
	public void done() {

	}

	@Override
	public void start() {
		if (mVisionTracker.isVisionEnabled()) {
			double degToTurn = mVisionTracker.getSkewFactor() * kTurretSkewFactor + Turret.convertRotationsToTurretDegrees(mTurret.getSetpoint());
			mTurret.setTurretPosition(Turret.convertTurretDegreesToRotations(degToTurn));
		}
	}
}
