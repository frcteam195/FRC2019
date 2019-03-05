package com.team195.frc2019.auto.actions;

import com.team195.frc2019.subsystems.HatchIntakeArm;

public class SetHatchArmRotationAction implements Action {
	private static final HatchIntakeArm mHatchArm = HatchIntakeArm.getInstance();

	private double mRotation;

	public SetHatchArmRotationAction(double armRotation) {
		mRotation = armRotation;
	}

	@Override
	public boolean isFinished() {
		return mHatchArm.isArmAtSetpoint(0.05);
	}

	@Override
	public void update() {
	}

	@Override
	public void done() {
	}

	@Override
	public void start() {
		mHatchArm.setHatchArmPosition(mRotation);
	}
}
