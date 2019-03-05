package com.team195.frc2019.auto.actions;

import com.team195.frc2019.subsystems.Elevator;
import com.team195.frc2019.subsystems.HatchIntakeArm;

public class SetHandoffCollisionAvoidanceAction implements Action {
	private boolean mOn;
	private static final Elevator mElevator = Elevator.getInstance();
	private static final HatchIntakeArm mHatchArm = HatchIntakeArm.getInstance();

	public SetHandoffCollisionAvoidanceAction(boolean on) {
		mOn = on;
	}

	@Override
	public boolean isFinished() {
		return true;
	}

	@Override
	public void update() {
	}

	@Override
	public void done() {

	}

	@Override
	public void start() {
		mElevator.setCollisionAvoidanceEnabled(mOn);
		mHatchArm.setCollisionAvoidanceEnabled(mOn);
	}
}
