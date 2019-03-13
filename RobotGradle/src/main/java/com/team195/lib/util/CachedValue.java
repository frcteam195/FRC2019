package com.team195.lib.util;

import com.team195.frc2019.reporters.ConsoleReporter;

import java.util.function.Function;

public class CachedValue<T> {
	private final TimeoutTimer mTimeoutTimer;

	private T mCachedValue;

	private boolean initialized = false;

	private Function<Void, T> mUpdateFunction;

	public CachedValue(double updateTimeoutMs, Function<Void, T> updateFunction) {
		mTimeoutTimer = new TimeoutTimer(updateTimeoutMs / 1000.0);
		mUpdateFunction = updateFunction;
	}

	public synchronized T getValue() {
		try {
			if (!initialized) {
				setCachedValue(mUpdateFunction.apply(null));
				initialized = true;
			}

			if (mTimeoutTimer.isTimedOut()) {
				setCachedValue(mUpdateFunction.apply(null));
				mTimeoutTimer.reset();
			}
		}
		catch (Exception ex) {
			ConsoleReporter.report(ex);
		}

		return mCachedValue;
	}

	public synchronized void setValue(T value) {
		setCachedValue(value);
	}

	private synchronized void setCachedValue(T value) {
		mCachedValue = value;
	}
}
