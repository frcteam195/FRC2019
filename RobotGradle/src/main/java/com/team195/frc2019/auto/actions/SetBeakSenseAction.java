package com.team195.frc2019.auto.actions;

import com.team195.frc2019.constants.AutoConstants;
import com.team195.frc2019.subsystems.Turret;
import com.team195.lib.util.TimeoutTimer;

public class SetBeakSenseAction implements Action {
	private static final Turret mTurret = Turret.getInstance();
	private final TimeoutTimer mTimeoutTimer = new TimeoutTimer(AutoConstants.kDefaultSolenoidWait * 2.0);

	private boolean mOn;

	public SetBeakSenseAction(boolean beakSenseOn) {
		mOn = beakSenseOn;
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
		mTurret.setBeakListened(mOn);
	}
}
