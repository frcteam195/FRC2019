package com.team195.frc2019.auto.actions;

import com.team195.frc2019.auto.AutoConstants;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.subsystems.Turret;
import com.team195.lib.util.TimeoutTimer;

public class WaitForHatchOrTimeoutAction implements Action {
	private static final Turret mTurret = Turret.getInstance();

	private final TimeoutTimer mTimeoutTimer;


	public WaitForHatchOrTimeoutAction() {
		this(AutoConstants.kLimitSwitchTriggerTimeout);
	}

	public WaitForHatchOrTimeoutAction(double timeout) {
		mTimeoutTimer = new TimeoutTimer(timeout);
	}

	@Override
	public boolean isFinished() {
		return (mTimeoutTimer.isTimedOut() || !mTurret.getLimitSwitchValue());
	}

	@Override
	public void update() {
	}

	@Override
	public void done() {

	}

	@Override
	public void start() {

	}
}
