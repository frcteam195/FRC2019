package com.team195.frc2019.auto.autonomy;

import com.team195.frc2019.auto.actions.Action;
import com.team195.frc2019.subsystems.Subsystem;

import java.util.Arrays;
import java.util.HashSet;

public class AutomatedAction implements Action {

	private HashSet<Subsystem> requiredSubsystems = new HashSet<>();
	private Action mAction;

	public static AutomatedAction fromAction(Action action, Subsystem... requirements) {
		AutomatedAction a = new AutomatedAction(action);
		a.addRequirements(requirements);
		return a;
	}

	public AutomatedAction(Action action) {
		mAction = action;
	}

	public void addRequirements(Subsystem... subsystems) {
		requiredSubsystems.addAll(Arrays.asList(subsystems));
	}

	@Override
	public boolean isFinished() {
		return mAction.isFinished();
	}

	@Override
	public void update() {
		mAction.update();
	}

	@Override
	public void done() {
		mAction.done();
	}

	@Override
	public void start() {
		mAction.start();
	}
}
