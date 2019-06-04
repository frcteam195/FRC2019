package com.team195.frc2019.auto.actions;

import com.team195.frc2019.constants.AutoConstants;
import com.team195.lib.util.TimeoutTimer;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;
import java.util.function.Predicate;

public class WaitForFallingEdgeButtonAction implements Action {
	private final TimeoutTimer mTimeoutTimer;

	private final AtomicReference<Predicate<Void>> mButtonGetterMethod;

	private boolean prevButtonVal = true;


	public WaitForFallingEdgeButtonAction(AtomicReference<Predicate<Void>> buttonGetterMethod) {
		this(buttonGetterMethod, AutoConstants.kDefaultButtonTimeout);
	}

	public WaitForFallingEdgeButtonAction(AtomicReference<Predicate<Void>> buttonGetterMethod, double timeout) {
		mButtonGetterMethod = buttonGetterMethod;
		mTimeoutTimer = new TimeoutTimer(timeout);
	}

	@Override
	public boolean isFinished() {
		if (mButtonGetterMethod.get() != null) {
			boolean currentInput = mButtonGetterMethod.get().test(null);
			boolean retVal = (currentInput != prevButtonVal) && !currentInput;
			prevButtonVal = currentInput;
			return (mTimeoutTimer.isTimedOut() || retVal);
		}
		else
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
		mTimeoutTimer.reset();
	}
}
