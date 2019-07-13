package com.team195.frc2019.auto.modes;

import com.team195.frc2019.auto.AutoModeBase;
import com.team195.frc2019.auto.AutoModeEndedException;
import com.team195.frc2019.auto.actions.Action;
import com.team195.frc2019.auto.actions.ParallelAction;
import com.team195.frc2019.auto.actions.SetDriveBaseVelocity;
import com.team195.frc2019.auto.actions.WaitAction;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;

import java.util.ArrayList;

public class TestVelocityMode extends AutoModeBase {
	@Override
	protected void routine() throws AutoModeEndedException {
		// Inches/sec
//    	double[] speeds = {30, 90, 140, 90, 30, 0};
		double[] speeds = {30, 90, 90, 90, 30, 0};
		// sec
		double[] delays = {.3, 	.3,	.3,	.3,	.3, .3};

//		assert(speeds.length == delays.length);

		for(int i = 0; i < speeds.length; i++) {
			ConsoleReporter.report("Running velocity " + i);
			ArrayList<Action> currentAction = new ArrayList<Action>();
			currentAction.add(new SetDriveBaseVelocity(speeds[i], speeds[i]));
			currentAction.add(new WaitAction(delays[i]));
			runAction(new ParallelAction(currentAction));
		}

	}

	@Override
	public Pose2d getStartingCoords() {
		return new Pose2d(0, 0, Rotation2d.identity());
	}
}