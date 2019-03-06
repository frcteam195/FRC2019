package com.team195.frc2019.auto.actions;

import com.team195.frc2019.subsystems.HatchIntakeArm;

public class SetHatchArmRotationAction implements Action {
	private static final HatchIntakeArm mHatchArm = HatchIntakeArm.getInstance();

	private double mRotation;
	private double mPosDelta;

	public SetHatchArmRotationAction(double armRotation) {
		this(armRotation, 0.05);
	}

	public SetHatchArmRotationAction(double armRotation, double posDelta) {
		mRotation = armRotation;
		mPosDelta = posDelta;
	}


	@Override
	public boolean isFinished() {
		return mHatchArm.isArmAtSetpoint(mPosDelta);
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
