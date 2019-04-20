package com.team195.frc2019.auto.actions;

import com.team195.frc2019.subsystems.Turret;
import com.team195.frc2019.subsystems.VisionTracker;
import com.team195.lib.util.TimeoutTimer;

import java.util.function.Function;

public class SetTurretAutoSkewAction implements Action {
	private static final Turret mTurret = Turret.getInstance();
	private static final VisionTracker mVisionTracker = VisionTracker.getInstance();
	private final TimeoutTimer mTimeoutTimer = new TimeoutTimer(15);

	private static final double kTurretSkewFactor = 3;

	Function<Void, Boolean> mButtonGetterMethod;

	public SetTurretAutoSkewAction(Function<Void, Boolean> buttonGetterMethod) {
		mButtonGetterMethod = buttonGetterMethod;
	}

	@Override
	public boolean isFinished() {
		return mTimeoutTimer.isTimedOut()
				|| !mButtonGetterMethod.apply(null);
	}

	@Override
	public void update() {
	}

	@Override
	public void done() {
		double degToTurn = mVisionTracker.getSkewFactor() * kTurretSkewFactor + Turret.convertRotationsToTurretDegrees(mTurret.getSetpoint());
		mTurret.setTurretPosition(Turret.convertTurretDegreesToRotations(degToTurn));
		mVisionTracker.setVisionEnabled(false);
	}

	@Override
	public void start() {
		mVisionTracker.setTargetMode(VisionTracker.TargetMode.HATCH_AUTOSKEW);
		mVisionTracker.setVisionEnabled(true);
	}
}
