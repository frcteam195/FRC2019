package com.team195.frc2019.auto.actions;

import com.team195.frc2019.constants.AutoConstants;
import com.team195.lib.util.TimeoutTimer;

import java.util.function.Function;

public class WaitForFallingEdgeButtonAction implements Action {
	private final TimeoutTimer mTimeoutTimer;

	private final Function<Void, Boolean> mButtonGetterMethod;

	private boolean prevButtonVal = true;


	public WaitForFallingEdgeButtonAction(Function<Void, Boolean> buttonGetterMethod) {
		this(buttonGetterMethod, AutoConstants.kDefaultButtonTimeout);
	}

	public WaitForFallingEdgeButtonAction(Function<Void, Boolean> buttonGetterMethod, double timeout) {
		mButtonGetterMethod = buttonGetterMethod;
		mTimeoutTimer = new TimeoutTimer(timeout);
	}

	@Override
	public boolean isFinished() {
		boolean currentInput = mButtonGetterMethod.apply(null);
		boolean retVal = (currentInput != prevButtonVal) && !currentInput;
		prevButtonVal = currentInput;
		return (mTimeoutTimer.isTimedOut() || retVal);
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
