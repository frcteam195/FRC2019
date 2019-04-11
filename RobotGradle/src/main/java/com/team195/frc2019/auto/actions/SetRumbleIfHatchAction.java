package com.team195.frc2019.auto.actions;

import com.team195.frc2019.subsystems.Turret;
import com.team195.lib.drivers.dashjoy.CKDashJoystick;
import com.team195.lib.util.TimeoutTimer;

import java.util.function.Function;

public class SetRumbleIfHatchAction implements Action {
	private final TimeoutTimer mTimeoutTimer = new TimeoutTimer(1);
	private static final Turret mTurret = Turret.getInstance();

	private double mVal;
	private final Function<Void, CKDashJoystick> mDriveJoystickGetter;

	public SetRumbleIfHatchAction(Function<Void, CKDashJoystick> driveJoystickGetter, double val) {
		mDriveJoystickGetter = driveJoystickGetter;
		mVal = val;
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
		mDriveJoystickGetter.apply(null).setRumble(0);
	}

	@Override
	public void start() {
		if (!mTurret.getLimitSwitchValue())
			mDriveJoystickGetter.apply(null).setRumble(mVal);
	}
}
