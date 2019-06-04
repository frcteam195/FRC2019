package com.team195.frc2019.auto.actions;

import com.team195.frc2019.subsystems.Turret;
import com.team195.lib.drivers.dashjoy.CKDashJoystick;
import com.team195.lib.util.TimeoutTimer;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;

public class SetRumbleIfHatchAction implements Action {
	private final TimeoutTimer mTimeoutTimer = new TimeoutTimer(1);
	private static final Turret mTurret = Turret.getInstance();

	private double mVal;
	private final AtomicReference<Function<Void, CKDashJoystick>> mDriveJoystickGetter;

	public SetRumbleIfHatchAction(AtomicReference<Function<Void, CKDashJoystick>> driveJoystickGetter, double val) {
		mDriveJoystickGetter = driveJoystickGetter;
		mVal = val;
	}

	@Override
	public boolean isFinished() {
		if (mDriveJoystickGetter.get() != null)
			return mTimeoutTimer.isTimedOut();
		else
			return true;
	}

	@Override
	public void update() {
	}

	@Override
	public void done() {
		if (mDriveJoystickGetter.get() != null)
			mDriveJoystickGetter.get().apply(null).setRumble(0);
	}

	@Override
	public void start() {
		mTimeoutTimer.reset();
		if (!mTurret.getLimitSwitchValue()) {
			if (mDriveJoystickGetter.get() != null)
				mDriveJoystickGetter.get().apply(null).setRumble(mVal);
		}
	}
}
