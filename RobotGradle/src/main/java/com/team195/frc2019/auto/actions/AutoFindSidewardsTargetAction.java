package com.team195.frc2019.auto.actions;

import com.team195.frc2019.subsystems.Drive;
import com.team195.frc2019.subsystems.Turret;
import com.team195.frc2019.subsystems.VisionTracker;
import com.team195.frc2019.subsystems.positions.TurretPositions;
import com.team254.lib.util.DriveSignal;

import java.util.function.Function;

public class AutoFindSidewardsTargetAction implements Action {
	private static final VisionTracker mVisionTracker = VisionTracker.getInstance();
	private static final Turret mTurret = Turret.getInstance();
	private static final Drive mDrive = Drive.getInstance();

	public AutoFindSidewardsTargetAction() {

	}

	@Override
	public boolean isFinished() {
		return mVisionTracker.getTargetHorizAngleDev() != 0 && mVisionTracker.getTargetHorizAngleDev() < 1.5;
	}

	@Override
	public void update() {
		double throttle = 0;
		if (VisionTracker.getInstance().isTargetFound()) {
			if (Turret.getInstance().getSetpoint() == TurretPositions.Right90) {
				throttle = -Math.max(Math.min(VisionTracker.getInstance().getTargetHorizAngleDev() * 0.01, 1), -1);
			} else if (Turret.getInstance().getSetpoint() == TurretPositions.Left90) {
				throttle = Math.max(Math.min(VisionTracker.getInstance().getTargetHorizAngleDev() * 0.01, 1), -1);
			}
		} else {
			throttle = 0.25;
		}
		mDrive.setOpenLoopAutomated(new DriveSignal(throttle, throttle));
	}

	@Override
	public void done() {
		mDrive.setOpenLoopAutomated(DriveSignal.NEUTRAL);
		mVisionTracker.setVisionEnabled(false);
	}

	@Override
	public void start() {
		mDrive.setBrakeMode(true);
		mDrive.forceBrakeModeUpdate();
		mVisionTracker.setTargetMode(VisionTracker.TargetMode.HATCH);
		mVisionTracker.setVisionEnabled(true);
	}
}
