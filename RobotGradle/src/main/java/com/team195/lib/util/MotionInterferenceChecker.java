package com.team195.lib.util;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.Function;

public class MotionInterferenceChecker {
	private ArrayList<Function<Void, Boolean>> conditionList = new ArrayList<>();
	private LogicOperation mLogicOperation = LogicOperation.AND;

	@SafeVarargs
	public MotionInterferenceChecker(LogicOperation logicOperation, Function<Void, Boolean>... conditions) {
		mLogicOperation = logicOperation;
		conditionList.addAll(Arrays.asList(conditions));
	}

	@SafeVarargs
	public final void add(Function<Void, Boolean>... conditions) {
		conditionList.addAll(Arrays.asList(conditions));
	}

	public boolean hasPassedConditions() {
		if (mLogicOperation == LogicOperation.OR)
			return conditionList.stream().map(f -> f.apply(null)).reduce(true, (a, b) -> a && b);
		else
			return conditionList.stream().map(f -> f.apply(null)).reduce(true, (a, b) -> a || b);
	}

	public enum LogicOperation {
		AND,
		OR;
	}
}
