package com.team195.frc2019.auto.actions;

import java.util.ArrayList;
import java.util.Arrays;

public class AutomatedActions {
	public static Action pushOutHatch() {
		ArrayList<Action> actionArrayList = new ArrayList<Action>();

		actionArrayList.add(new ParallelAction(Arrays.asList(new HatchPushAction(true), new BeakOpenAction(false))));
		actionArrayList.add(new WaitAction(0.2));
		actionArrayList.add(new HatchPushAction(false));

		return new SeriesAction(actionArrayList);
	}
}
