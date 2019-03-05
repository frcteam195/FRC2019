package com.team195.frc2019.auto.actions;

import com.team195.frc2019.auto.AutoConstants;
import com.team195.frc2019.subsystems.positions.BallIntakeArmPositions;
import com.team195.frc2019.subsystems.positions.ElevatorPositions;

import java.util.ArrayList;
import java.util.Arrays;

public class AutomatedActions {
	private static boolean unfolded = false;

	public static boolean isUnfolded() {
		return unfolded;
	}

	public static Action unfold() {
		ArrayList<Action> actionArrayList = new ArrayList<>();

		actionArrayList.add(new SetBallArmRotationAction(BallIntakeArmPositions.Down));
		actionArrayList.add(new WaitAction(AutoConstants.kWaitForArmFall));
		actionArrayList.add(new SetElevatorHeightAction(ElevatorPositions.RocketHatchLow));

		return new SeriesAction(actionArrayList);
	}

	public static Action pushOutHatch() {
		ArrayList<Action> actionArrayList = new ArrayList<>();

		actionArrayList.add(new ParallelAction(Arrays.asList(new HatchPushAction(true), new BeakOpenAction(false))));
		actionArrayList.add(new WaitAction(AutoConstants.kWaitForHatchPush));
		actionArrayList.add(new HatchPushAction(false));

		return new SeriesAction(actionArrayList);
	}

	public static Action shootBall(double wheelRPM) {
		ArrayList<Action> actionArrayList = new ArrayList<>();

		actionArrayList.add(new SpinUpBallShooter(wheelRPM));
		actionArrayList.add(new BallPushAction(true));
		actionArrayList.add(new WaitAction(AutoConstants.kWaitForBalLPush));
		actionArrayList.add(new BallPushAction(false));

		return new SeriesAction(actionArrayList);
	}
}