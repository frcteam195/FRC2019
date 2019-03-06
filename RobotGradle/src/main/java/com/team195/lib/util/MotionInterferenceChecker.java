package com.team195.lib.util;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.Function;

public class MotionInterferenceChecker {
	private ArrayList<Function<Void, Boolean>> conditionList = new ArrayList<>();
	private LogicOperation mLogicOperation = LogicOperation.AND;

	private boolean cachingEnabled = false;

	private final CachedValue<Boolean> mCachedValue = new CachedValue<>(100, (t) -> checkLogic());

	private boolean enabled = true;

	@SafeVarargs
	public MotionInterferenceChecker(LogicOperation logicOperation, Function<Void, Boolean>... conditions) {
		this(logicOperation, false, conditions);
	}

	@SafeVarargs
	public MotionInterferenceChecker(LogicOperation logicOperation, boolean cachingEnabled, Function<Void, Boolean>... conditions) {
		mLogicOperation = logicOperation;
		conditionList.addAll(Arrays.asList(conditions));
		this.cachingEnabled = cachingEnabled;
	}

	@SafeVarargs
	public final void add(Function<Void, Boolean>... conditions) {
		conditionList.addAll(Arrays.asList(conditions));
	}

	public synchronized void setEnabled(boolean enabled) {
		this.enabled = enabled;
	}

	public boolean isEnabled() {
		return enabled;
	}

	public boolean hasPassedConditions() {
		if (cachingEnabled) {
			return mCachedValue.getValue();
		}
		else
			return checkLogic();
	}

	private boolean checkLogic() {
		if (mLogicOperation == LogicOperation.OR)
			return conditionList.stream().map(f -> f.apply(null)).reduce(false, (a, b) -> a || b);
		else
			return conditionList.stream().map(f -> f.apply(null)).reduce(true, (a, b) -> a && b);
	}

	public enum LogicOperation {
		AND,
		OR;
	}
}
