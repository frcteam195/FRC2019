package com.team195.lib.util;

import java.util.function.Function;

public class CachedValue<T> {
	private final TimeoutTimer mTimeoutTimer;

	private T mCachedValue;

	private Function<Void, T> mUpdateFunction;

	public CachedValue(double updateTimeoutMs, Function<Void, T> updateFunction) {
		mTimeoutTimer = new TimeoutTimer(updateTimeoutMs / 1000.0);
		mUpdateFunction = updateFunction;
	}

	public synchronized T getValue() {
		if (mTimeoutTimer.isTimedOut()) {
			mCachedValue = mUpdateFunction.apply(null);
		}
		return mCachedValue;
	}
}
