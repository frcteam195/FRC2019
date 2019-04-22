package com.team195.frc2019.auto.actions;

import com.team195.frc2019.subsystems.Drive;

public class ForceTerminatePathAction implements Action {
	private static final Drive mDrive = Drive.getInstance();

	public ForceTerminatePathAction() {
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
		mDrive.forceDoneWithPath();
	}
}

