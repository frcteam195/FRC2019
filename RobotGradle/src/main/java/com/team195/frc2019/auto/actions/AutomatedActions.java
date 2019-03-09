package com.team195.frc2019.auto.actions;

import com.team195.frc2019.auto.AutoConstants;
import com.team195.frc2019.subsystems.BallIntakeArm;
import com.team195.frc2019.subsystems.Turret;
import com.team195.frc2019.subsystems.positions.BallIntakeArmPositions;
import com.team195.frc2019.subsystems.positions.ElevatorPositions;
import com.team195.frc2019.subsystems.positions.HatchArmPositions;
import com.team195.frc2019.subsystems.positions.TurretPositions;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.Function;

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

	public static Action pickupHatchFeederStation() {
		ArrayList<Action> actionArrayList = new ArrayList<>();

		actionArrayList.add(new ParallelAction(Arrays.asList(new SetElevatorHeightAction(ElevatorPositions.HatchPickupStation),
							new SetBeakAction(false))));
		actionArrayList.add(new SetHatchPushAction(true));
		actionArrayList.add(new WaitForHatchOrTimeoutAction());
		actionArrayList.add(new SetHatchPushAction(false));

		return new SeriesAction(actionArrayList);
	}

	public static Action hatchHandoff() {
		ArrayList<Action> actionArrayList = new ArrayList<>();
		actionArrayList.add(new SetHandoffCollisionAvoidanceAction(false));
		actionArrayList.add(new ParallelAction(Arrays.asList(new SetElevatorHeightAction(ElevatorPositions.HatchHandoff), new SetBeakAction(false))));
		actionArrayList.add(new ParallelAction(Arrays.asList(new SetHatchArmRollerAction(0.3), new SetHatchArmRotationAction(HatchArmPositions.Handoff, 0.15))));
		actionArrayList.add(new SetHatchPushAction(true));
		actionArrayList.add(new ParallelAction(Arrays.asList(new SetElevatorHeightAction(ElevatorPositions.HatchHandoff + ElevatorPositions.HatchLiftOffset),
				                                             new SetHatchArmRollerAction(-0.2))));
		actionArrayList.add(new ParallelAction(Arrays.asList(new SetHatchArmRollerAction(0),
															 new SetHatchArmRotationAction(HatchArmPositions.Inside, 0.15),
															 new SetHatchPushAction(false))));
		actionArrayList.add(new SetHandoffCollisionAvoidanceAction(true));
		return new SeriesAction(actionArrayList);
	}

	public static Action placeHatch(double elevatorPosition, Function<Void, Boolean> buttonValueGetter) {
		ArrayList<Action> actionArrayList = new ArrayList<>();

		actionArrayList.add(new SetElevatorHeightAction(elevatorPosition));
		actionArrayList.add(new WaitForFallingEdgeButtonAction(buttonValueGetter, 100));
		actionArrayList.add(new SetHatchPushAction(true));
		actionArrayList.add(new WaitAction(AutoConstants.kWaitForHatchPush));
		actionArrayList.add(new SetBeakAction(false));
		actionArrayList.add(new SetHatchPushAction(false));
		actionArrayList.add(new WaitAction(AutoConstants.kWaitForHatchPush * 2));
		actionArrayList.add(new ParallelAction(Arrays.asList(new SetElevatorHeightAction(ElevatorPositions.Resting),
				new SetTurretPositionAction(TurretPositions.Home))));

		return new SeriesAction(actionArrayList);
	}

	public static Action shootBall(double elevatorPosition, Function<Void, Boolean> buttonValueGetter) {
		ArrayList<Action> actionArrayList = new ArrayList<>();

		actionArrayList.add(new SetElevatorHeightAction(elevatorPosition));
		actionArrayList.add(new WaitForFallingEdgeButtonAction(buttonValueGetter, 100));
		actionArrayList.add(new SetBallShooterOpenLoopAction(TurretPositions.BallShootSpeedNormal));
		actionArrayList.add(new SetBallPushAction(true));
		actionArrayList.add(new WaitAction(AutoConstants.kWaitForBallPush));
		actionArrayList.add(new SetBallPushAction(false));
		actionArrayList.add(new SetBallShooterOpenLoopAction(0));
		actionArrayList.add(new ParallelAction(Arrays.asList(new SetElevatorHeightAction(ElevatorPositions.Resting),
				new SetTurretPositionAction(TurretPositions.Home))));

		return new SeriesAction(actionArrayList);
	}



	public static Action intakeBallOn(Function<Void, Boolean> buttonValueGetter) {
		ArrayList<Action> actionArrayList = new ArrayList<>();

		actionArrayList.add(new SetElevatorHeightAction(ElevatorPositions.Down));
		actionArrayList.add(new ParallelAction(Arrays.asList(new SetBallShooterOpenLoopAction(TurretPositions.BallShootSpeedIntake),
															 new SetBallIntakeAction(BallIntakeArmPositions.RollerIntake))));
		actionArrayList.add(new WaitForFallingEdgeButtonAction(buttonValueGetter, 20));
		actionArrayList.add(new ParallelAction(Arrays.asList(new SetBallShooterOpenLoopAction(TurretPositions.BallShootSpeedOff),
				new SetBallIntakeAction(BallIntakeArmPositions.RollerOff))));
		return new SeriesAction(actionArrayList);
	}

	public static Action intakeBallOff() {
		ArrayList<Action> actionArrayList = new ArrayList<>();

		actionArrayList.add(new ParallelAction(Arrays.asList(new SetBallShooterOpenLoopAction(0),
				new SetBallIntakeAction(0))));

		return new SeriesAction(actionArrayList);
	}

	public static Action prepareClimb() {
		ArrayList<Action> actionArrayList = new ArrayList<>();

		actionArrayList.add(new ParallelAction(Arrays.asList(new SetBallArmRotationAction(BallIntakeArmPositions.Up),
				new DropBallArmClimbBarAction())));

		return new SeriesAction(actionArrayList);
	}

	public static Action rollerHatchFloorIntake(Function<Void, Boolean> buttonValueGetter) {
		ArrayList<Action> actionArrayList = new ArrayList<>();

		actionArrayList.add(new ParallelAction(Arrays.asList(new SetHatchArmRollerAction(HatchArmPositions.RollerIntake),
				new SetHatchArmRotationAction(HatchArmPositions.Outside))));
		actionArrayList.add(new WaitForFallingEdgeButtonAction(buttonValueGetter, 10));
		actionArrayList.add(hatchHandoff());

		return new SeriesAction(actionArrayList);
	}

	public static Action hatchArmOutIntake(Function<Void, Boolean> buttonValueGetter) {
		ArrayList<Action> actionArrayList = new ArrayList<>();

		actionArrayList.add(new ParallelAction(Arrays.asList(new SetHatchArmRollerAction(HatchArmPositions.RollerOuttake),
				new SetHatchArmRotationAction(HatchArmPositions.Outside))));
		actionArrayList.add(new WaitForFallingEdgeButtonAction(buttonValueGetter, 10));
		actionArrayList.add(new ParallelAction(Arrays.asList(new SetHatchArmRollerAction(HatchArmPositions.RollerOff),
				new SetHatchArmRotationAction(HatchArmPositions.Inside))));


		return new SeriesAction(actionArrayList);
	}

	public static Action ballOuttake(Function<Void, Boolean> buttonValueGetter) {
		ArrayList<Action> actionArrayList = new ArrayList<>();

		actionArrayList.add(new ParallelAction(Arrays.asList(new SetBallIntakeAction(BallIntakeArmPositions.RollerOuttake),
				new SetBallShooterOpenLoopAction(TurretPositions.BallShootSpeedNormal))));
		actionArrayList.add(new WaitForFallingEdgeButtonAction(buttonValueGetter, 10));
		actionArrayList.add(new ParallelAction(Arrays.asList(new SetBallIntakeAction(BallIntakeArmPositions.RollerOff),
				new SetBallShooterOpenLoopAction(TurretPositions.BallShootSpeedOff))));

		return new SeriesAction(actionArrayList);
	}

}