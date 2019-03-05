package com.team195.frc2019.auto.actions;

import com.team195.frc2019.auto.AutoConstants;
import com.team195.frc2019.subsystems.HatchIntakeArm;
import com.team195.lib.util.TimeoutTimer;

public class SetHatchArmRollerAction implements Action {
	private static final HatchIntakeArm mHatchArm = HatchIntakeArm.getInstance();

	private final TimeoutTimer mTimeoutTimer = new TimeoutTimer(AutoConstants.kDefaultRollerSpinUpWait);


	private double mSpeed;

	public SetHatchArmRollerAction(double rollerSpeed) {
		mSpeed = rollerSpeed;
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
		mHatchArm.setHatchRollerSpeed(mSpeed);
	}
}
