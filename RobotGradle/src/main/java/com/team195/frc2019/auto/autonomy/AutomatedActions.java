package com.team195.frc2019.auto.autonomy;

import com.team195.frc2019.constants.Constants;
import com.team195.frc2019.constants.AutoConstants;
import com.team195.frc2019.auto.actions.*;
import com.team195.frc2019.auto.actions.climb.SetClimbRackDownAction;
import com.team195.frc2019.auto.actions.climb.SetClimbRackUpAction;
import com.team195.frc2019.auto.actions.climb.SetDriveRampDownPowerAction;
import com.team195.frc2019.auto.actions.climb.SetIntakeBarForwardClimbAction;
import com.team195.frc2019.subsystems.*;
import com.team195.frc2019.subsystems.positions.BallIntakeArmPositions;
import com.team195.frc2019.subsystems.positions.ElevatorPositions;
import com.team195.frc2019.subsystems.positions.TurretPositions;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.Function;

public class AutomatedActions {
	public static AutomatedAction homeElevator() {
		ArrayList<Action> actionArrayList = new ArrayList<>();

		actionArrayList.add(new SetElevatorOpenLoopAction(-0.25));
		actionArrayList.add(new WaitForElevatorLimitAction(3));
		actionArrayList.add(new SetElevatorHomeAction());
		actionArrayList.add(new SetLEDAction(Constants.kGotGamePieceColor));

		return AutomatedAction.fromAction(new SeriesAction(actionArrayList), Constants.kActionTimeoutS, Elevator.getInstance());
	}

	public static AutomatedAction climbOpen(Function<Void, Boolean> buttonGetterMethod, Function<Void, Double> axisFrontGetterMethod, Function<Void, Double> axisBackGetterMethod) {
		ArrayList<Action> actionArrayList = new ArrayList<>();

		actionArrayList.add(new SetDrivePTOAction(true));
//		actionList.add(new SetBallIntakeAction(1));
		actionArrayList.add(new SetOpenLoopDriveAction(buttonGetterMethod, axisBackGetterMethod, axisFrontGetterMethod));

		return AutomatedAction.fromAction(new SeriesAction(actionArrayList), 300, Drive.getInstance());
	}

	public static AutomatedAction climbAutomated(Function<Void, Boolean> buttonGetterMethod) {
		ArrayList<Action> actionArrayList = new ArrayList<>();

		BallIntakeArm.getInstance().configureClimbCurrentLimit();
		Drive.getInstance().configureClimbCurrentLimit();

		actionArrayList.add(new ParallelAction(Arrays.asList(new SetElevatorHeightAction(ElevatorPositions.Down),
				new SetTurretPositionAction(TurretPositions.Home))));
		actionArrayList.add(new DropBallArmClimbBarAction());
		actionArrayList.add(new SetBallArmRotationAction(BallIntakeArmPositions.Down));
		actionArrayList.add(new SetDrivePTOAction(true));
		actionArrayList.add(new ParallelAction(Arrays.asList(new SetBallIntakeAction(BallIntakeArmPositions.RollerIntake),
				new SetIntakeBarForwardClimbAction())));
		actionArrayList.add(new SetClimbRackDownAction(buttonGetterMethod));
//		actionArrayList.add(new WaitAction(0.2));
		actionArrayList.add(new SetClimbRackUpAction());
		actionArrayList.add(new SetBallIntakeAction(BallIntakeArmPositions.RollerOff));
		actionArrayList.add(new SetDrivePTOAction(false));
		actionArrayList.add(new SetDriveRampDownPowerAction(0.4));

		return AutomatedAction.fromAction(new SeriesAction(actionArrayList), 300, Drive.getInstance(), BallIntakeArm.getInstance());
	}

	public static AutomatedAction climbAutomatedLvl2(Function<Void, Boolean> buttonGetterMethod) {
		ArrayList<Action> actionArrayList = new ArrayList<>();

		actionArrayList.add(new ParallelAction(Arrays.asList(new SetElevatorHeightAction(ElevatorPositions.Down),
				new SetTurretPositionAction(TurretPositions.Home))));
		actionArrayList.add(new DropBallArmClimbBarAction());
		actionArrayList.add(new SetBallArmRotationAction(BallIntakeArmPositions.Down));
		actionArrayList.add(new SetDrivePTOAction(true));
		actionArrayList.add(new SetIntakeBarForwardClimbAction());
		actionArrayList.add(new SetBallIntakeAction(BallIntakeArmPositions.RollerIntake));
		actionArrayList.add(new WaitAction(0.2));
		actionArrayList.add(new SetDrivePTOAction(false));
		actionArrayList.add(new OpenLoopDrive(0.5, 0.5, 2));
		actionArrayList.add(new SetBallIntakeAction(BallIntakeArmPositions.RollerOff));


		return AutomatedAction.fromAction(new SeriesAction(actionArrayList), 300, Drive.getInstance(), BallIntakeArm.getInstance());
	}

	public static AutomatedAction reverseHatchPickup() {
		ArrayList<Action> actionArrayList = new ArrayList<>();

		actionArrayList.add(new SetHatchPushAction(false));

		if (Turret.getInstance().getSetpoint() != TurretPositions.Back180 || Elevator.getInstance().getSetpoint() != ElevatorPositions.HatchPickupStation) {
			actionArrayList.add(new SetBallArmRotationAction(BallIntakeArmPositions.Down));
			actionArrayList.add(new ParallelAction(Arrays.asList(new SetElevatorHeightAction(ElevatorPositions.HatchPickupStation),
					new SeriesAction(Arrays.asList(new WaitForElevatorGreaterThanPositionAction(ElevatorPositions.CollisionThresholdTurret, 1),
							new SetTurretPositionAction(TurretPositions.Back180))))));
			actionArrayList.add(new SetBallArmRotationAction(BallIntakeArmPositions.Up));
		}
		actionArrayList.add(new SetBeakAction(false));
		actionArrayList.add(new SetHatchPushAction(true));
		actionArrayList.add(new WaitForHatchOrTimeoutAction());
		actionArrayList.add(new SetElevatorHeightAction(ElevatorPositions.HatchPickupStationLift));
		actionArrayList.add(new WaitAction(0.2));
		actionArrayList.add(new SetHatchPushAction(false));
		actionArrayList.add(new WaitAction(0.2));
		actionArrayList.add(new SetElevatorHeightAction(ElevatorPositions.RocketHatchLow));

		return AutomatedAction.fromAction(new SeriesAction(actionArrayList), Constants.kActionTimeoutS, Elevator.getInstance(), Turret.getInstance(), BallIntakeArm.getInstance());
	}

	public static AutomatedAction reverseHatchPlaceLow() {
		ArrayList<Action> actionArrayList = new ArrayList<>();

		actionArrayList.add(new SetHatchPushAction(false));

		if (Turret.getInstance().getSetpoint() != TurretPositions.Back180 || Elevator.getInstance().getSetpoint() != ElevatorPositions.RocketHatchLow) {

			if (Elevator.getInstance().getPosition() > ElevatorPositions.CargoBall)
				actionArrayList.add(new SetBallArmRotationAction(BallIntakeArmPositions.Down));

			actionArrayList.add(new ParallelAction(Arrays.asList(new SetElevatorHeightAction(ElevatorPositions.RocketHatchLow),
					new SeriesAction(Arrays.asList(new WaitForElevatorGreaterThanPositionAction(ElevatorPositions.CollisionThresholdTurret, 1),
							new SetTurretPositionAction(TurretPositions.Back180))))));
			actionArrayList.add(new SetBallArmRotationAction(BallIntakeArmPositions.Up));
		}

		return AutomatedAction.fromAction(new SeriesAction(actionArrayList), Constants.kActionTimeoutS, Elevator.getInstance(), Turret.getInstance(), BallIntakeArm.getInstance());
	}

	public static AutomatedAction reverseHatchPlaceMid() {
		ArrayList<Action> actionArrayList = new ArrayList<>();

		actionArrayList.add(new SetHatchPushAction(false));

		if (Turret.getInstance().getSetpoint() != TurretPositions.Back180 || Elevator.getInstance().getSetpoint() != ElevatorPositions.RocketHatchMed) {

			if (Elevator.getInstance().getPosition() <= ElevatorPositions.CollisionThresholdBallArm)
				actionArrayList.add(new SetBallArmRotationAction(BallIntakeArmPositions.Down));

			actionArrayList.add(new ParallelAction(Arrays.asList(new SetElevatorHeightAction(ElevatorPositions.RocketHatchMed),
					new SeriesAction(Arrays.asList(new WaitForElevatorGreaterThanPositionAction(ElevatorPositions.CollisionThresholdTurret, 1),
							new SetTurretPositionAction(TurretPositions.Back180))))));
		    actionArrayList.add(new SetBallArmRotationAction(BallIntakeArmPositions.Up));
		}

		return AutomatedAction.fromAction(new SeriesAction(actionArrayList), Constants.kActionTimeoutS, Elevator.getInstance(), Turret.getInstance(), BallIntakeArm.getInstance());
	}

	public static AutomatedAction reverseHatchPlaceHigh() {
		ArrayList<Action> actionArrayList = new ArrayList<>();

		actionArrayList.add(new SetHatchPushAction(false));

		if (Turret.getInstance().getSetpoint() != TurretPositions.Back180 || Elevator.getInstance().getSetpoint() != ElevatorPositions.RocketHatchHigh) {

			if (Elevator.getInstance().getPosition() <= ElevatorPositions.CollisionThresholdBallArm)
				actionArrayList.add(new SetBallArmRotationAction(BallIntakeArmPositions.Down));

			actionArrayList.add(new ParallelAction(Arrays.asList(new SetElevatorHeightAction(ElevatorPositions.RocketHatchHigh),
					new SeriesAction(Arrays.asList(new WaitForElevatorGreaterThanPositionAction(ElevatorPositions.CollisionThresholdTurret, 1),
							new SetTurretPositionAction(TurretPositions.Back180))))));
			actionArrayList.add(new SetBallArmRotationAction(BallIntakeArmPositions.Up));
		}

		return AutomatedAction.fromAction(new SeriesAction(actionArrayList), Constants.kActionTimeoutS, Elevator.getInstance(), Turret.getInstance(), BallIntakeArm.getInstance());
	}

	public static AutomatedAction lowerIntakeAndResetTurret() {
		ArrayList<Action> actionArrayList = new ArrayList<>();

		actionArrayList.add(new SetHatchPushAction(false));

		actionArrayList.add(new SetBallArmRotationAction(BallIntakeArmPositions.Down));
		actionArrayList.add(new SetTurretPositionAction(TurretPositions.Home));

		return AutomatedAction.fromAction(new SeriesAction(actionArrayList), Constants.kActionTimeoutS, BallIntakeArm.getInstance(), Turret.getInstance());
	}

	public static AutomatedAction unfold() {
		ArrayList<Action> actionArrayList = new ArrayList<>();

		actionArrayList.add(new SetHatchPushAction(false));

		if (BallIntakeArm.getInstance().isArmUp()) {
			actionArrayList.add(new ParallelAction(Arrays.asList(new SetBallArmRotationAction(BallIntakeArmPositions.Down),
					new SetElevatorHeightAction(ElevatorPositions.RocketHatchLow))));
		}
		else {
			actionArrayList.add(new SetElevatorHeightAction(ElevatorPositions.RocketHatchLow));
		}

		return AutomatedAction.fromAction(new SeriesAction(actionArrayList), Constants.kActionTimeoutS, BallIntakeArm.getInstance(), Elevator.getInstance());
	}

	public static AutomatedAction pickupHatchFeederStation() {
		ArrayList<Action> actionArrayList = new ArrayList<>();

		actionArrayList.add(new SetHatchPushAction(false));

		if (BallIntakeArm.getInstance().getSetpoint() != BallIntakeArmPositions.Down)
			actionArrayList.add(new SetBallArmRotationAction(BallIntakeArmPositions.Down));

		actionArrayList.add(new ParallelAction(Arrays.asList(new SetElevatorHeightAction(ElevatorPositions.HatchPickupStation),
							new SetBeakAction(false),
				new SeriesAction(Arrays.asList(new WaitForElevatorGreaterThanPositionAction(ElevatorPositions.CollisionThresholdTurret, 1),
						new SetTurretPositionAction(TurretPositions.Home))))));
		actionArrayList.add(new SetBeakSenseAction(true));
		actionArrayList.add(new SetHatchPushAction(true));
		actionArrayList.add(new WaitForHatchOrTimeoutAction());
		actionArrayList.add(new SetElevatorHeightAction(ElevatorPositions.HatchPickupStationLift));
		actionArrayList.add(new WaitAction(0.2));
		actionArrayList.add(new SetHatchPushAction(false));
		actionArrayList.add(new WaitAction(0.2));
		actionArrayList.add(new SetElevatorHeightAction(ElevatorPositions.RocketHatchLow));

		return AutomatedAction.fromAction(new SeriesAction(actionArrayList), Constants.kActionTimeoutS, Elevator.getInstance(), Turret.getInstance());
	}

	public static AutomatedAction pickupHatchFeederStationDriveAway(Function<Void, Boolean> enableDriveAway) {
		ArrayList<Action> actionArrayList = new ArrayList<>();

		actionArrayList.add(new SetHatchPushAction(false));

		if (BallIntakeArm.getInstance().getSetpoint() != BallIntakeArmPositions.Down)
			actionArrayList.add(new SetBallArmRotationAction(BallIntakeArmPositions.Down));

		actionArrayList.add(new ParallelAction(Arrays.asList(new SetElevatorHeightAction(ElevatorPositions.HatchPickupStation),
				new SetBeakAction(false),
				new SeriesAction(Arrays.asList(new WaitForElevatorGreaterThanPositionAction(ElevatorPositions.CollisionThresholdTurret, 1),
						new SetTurretPositionAction(TurretPositions.Home))))));
		actionArrayList.add(new SetBeakSenseAction(true));
		actionArrayList.add(new SetHatchPushAction(true));
		actionArrayList.add(new WaitForHatchOrTimeoutAction());
		actionArrayList.add(new SetElevatorHeightAction(ElevatorPositions.HatchPickupStationLift));
		actionArrayList.add(new WaitAction(0.2));
		actionArrayList.add(new SetHatchPushAction(false));
		actionArrayList.add(new WaitAction(0.2));
		actionArrayList.add(new SetElevatorHeightAction(ElevatorPositions.RocketHatchLow));

		//Got Hatch
		if (enableDriveAway.apply(null) && !Turret.getInstance().getLimitSwitchValue())
			actionArrayList.add(new SetOpenLoopAutomatedDrive(1, 1, 0.5));

		return AutomatedAction.fromAction(new SeriesAction(actionArrayList), Constants.kActionTimeoutS, Elevator.getInstance(), Turret.getInstance());
	}

	public static AutomatedAction placeHatch() {
		ArrayList<Action> actionArrayList = new ArrayList<>();

		actionArrayList.add(new SetBeakSenseAction(false));
		actionArrayList.add(new SetHatchPushAction(true));
		actionArrayList.add(new WaitAction(AutoConstants.kWaitForHatchPush));
		actionArrayList.add(new SetBeakForcedClosedAction());
		actionArrayList.add(new WaitAction(AutoConstants.kWaitForHatchPush));
		actionArrayList.add(new SetHatchPushAction(false));
		actionArrayList.add(new WaitAction(2));
		actionArrayList.add(new SetBeakAction(false));
		actionArrayList.add(new SetBeakSenseAction(true));

		return AutomatedAction.fromAction(new SeriesAction(actionArrayList), Constants.kActionTimeoutS, Turret.getInstance());
	}

	public static AutomatedAction shootBall() {
		ArrayList<Action> actionArrayList = new ArrayList<>();

		actionArrayList.add(new SetBallShooterOpenLoopAction(TurretPositions.BallShootSpeedNormal));
		actionArrayList.add(new SetBallPushAction(true));
		actionArrayList.add(new WaitAction(AutoConstants.kWaitForBallPush));
		actionArrayList.add(new SetBallPushAction(false));
		actionArrayList.add(new SetBallShooterOpenLoopAction(0));

		return AutomatedAction.fromAction(new SeriesAction(actionArrayList), Constants.kActionTimeoutS, Turret.getInstance());
	}

	public static AutomatedAction shootBall(Function<Void, Boolean> buttonValueGetter) {
		ArrayList<Action> actionArrayList = new ArrayList<>();

		actionArrayList.add(new SetBallShooterOpenLoopAction(TurretPositions.BallShootSpeedNormal));
		actionArrayList.add(new SetBallPushAction(true));
		actionArrayList.add(new WaitAction(AutoConstants.kWaitForBallPush));
		actionArrayList.add(new SetBallPushAction(false));
		actionArrayList.add(new WaitForFallingEdgeButtonAction(buttonValueGetter, 5));
		actionArrayList.add(new SetBallShooterOpenLoopAction(0));

		return AutomatedAction.fromAction(new SeriesAction(actionArrayList), Constants.kActionTimeoutS, Turret.getInstance());
	}

	public static AutomatedAction elevatorSet(double elevatorHeight) {
		ArrayList<Action> actionArrayList = new ArrayList<>();

		actionArrayList.add(new SetHatchPushAction(false));

		if (BallIntakeArm.getInstance().getSetpoint() != BallIntakeArmPositions.Down)
			actionArrayList.add(new SetBallArmRotationAction(BallIntakeArmPositions.Down));

		actionArrayList.add(new ParallelAction(Arrays.asList(new SetElevatorHeightAction(elevatorHeight),
							new SeriesAction(Arrays.asList(new WaitForElevatorGreaterThanPositionAction(ElevatorPositions.CollisionThresholdTurret, 1),
														   new SetTurretPositionAction(TurretPositions.Home))))));


		return AutomatedAction.fromAction(new SeriesAction(actionArrayList), Constants.kActionTimeoutS, Elevator.getInstance());
	}

	public static AutomatedAction ballArmSet(double armPosition) {
		ArrayList<Action> actionArrayList = new ArrayList<>();

		actionArrayList.add(new SetHatchPushAction(false));
		actionArrayList.add(new SetBallArmRotationAction(armPosition, 5));

		return AutomatedAction.fromAction(new SeriesAction(actionArrayList), Constants.kActionTimeoutS, BallIntakeArm.getInstance());
	}

	public static AutomatedAction intakeBallOn(Function<Void, Boolean> buttonValueGetter) {
		ArrayList<Action> actionArrayList = new ArrayList<>();

		actionArrayList.add(new SetHatchPushAction(false));

		if (BallIntakeArm.getInstance().getSetpoint() != BallIntakeArmPositions.Down)
			actionArrayList.add(new SetBallArmRotationAction(BallIntakeArmPositions.Down));

		actionArrayList.add(new ParallelAction(Arrays.asList(new SetTurretPositionAction(TurretPositions.Home),
				new SeriesAction(Arrays.asList(new WaitForTurretLessThanRotationAction(TurretPositions.Right90, 1),
						new SetElevatorHeightAction(ElevatorPositions.Down))))));

		actionArrayList.add(new ParallelAction(Arrays.asList(new SetBallShooterOpenLoopAction(TurretPositions.BallShootSpeedIntake),
															 new SetBallIntakeAction(BallIntakeArmPositions.RollerIntake))));
		actionArrayList.add(new WaitForFallingEdgeButtonAction(buttonValueGetter, 15));
		actionArrayList.add(new ParallelAction(Arrays.asList(new SetBallShooterOpenLoopAction(TurretPositions.BallShootSpeedOff),
				new SetBallIntakeAction(BallIntakeArmPositions.RollerOff))));
		return AutomatedAction.fromAction(new SeriesAction(actionArrayList), Constants.kActionTimeoutS, Elevator.getInstance(), BallIntakeArm.getInstance(), Turret.getInstance());
	}

	public static AutomatedAction intakeBallOff() {
		ArrayList<Action> actionArrayList = new ArrayList<>();

		actionArrayList.add(new ParallelAction(Arrays.asList(new SetBallShooterOpenLoopAction(0),
				new SetBallIntakeAction(0))));

		return AutomatedAction.fromAction(new SeriesAction(actionArrayList), Constants.kActionTimeoutS, BallIntakeArm.getInstance());
	}

	public static AutomatedAction enableHatchVision(Function<Void, Boolean> buttonValueGetter) {
		return AutomatedAction.fromAction(new SetVisionEnabledHatchSteerAction(buttonValueGetter), Constants.kActionTimeoutS, Drive.getInstance());
	}

	public static AutomatedAction enableHatchSidewaysVision(Function<Void, Boolean> buttonValueGetter) {
		return AutomatedAction.fromAction(new SetVisionEnabledHatchSidewaysAction(buttonValueGetter), Constants.kActionTimeoutS, Drive.getInstance());
	}

	public static AutomatedAction setTurretOpenLoop(Function<Void, Boolean> buttonGetterMethod, Function<Void, Double> axisGetterMethod) {
		return AutomatedAction.fromAction(new SetTurretOpenLoopAction(buttonGetterMethod, axisGetterMethod), Constants.kActionTimeoutS * 3, BallIntakeArm.getInstance());
	}

	public static AutomatedAction setTurretPosition(double turretPosition) {
		return AutomatedAction.fromAction(new SetTurretPositionAction(turretPosition), Constants.kActionTimeoutS, BallIntakeArm.getInstance());
	}

	public static AutomatedAction ballOuttake(Function<Void, Boolean> buttonValueGetter) {
		ArrayList<Action> actionArrayList = new ArrayList<>();

		actionArrayList.add(new ParallelAction(Arrays.asList(new SetBallIntakeAction(BallIntakeArmPositions.RollerOuttake),
				new SetBallShooterOpenLoopAction(TurretPositions.BallShootSpeedNormal))));
		actionArrayList.add(new WaitForFallingEdgeButtonAction(buttonValueGetter, 10));
		actionArrayList.add(new ParallelAction(Arrays.asList(new SetBallIntakeAction(BallIntakeArmPositions.RollerOff),
				new SetBallShooterOpenLoopAction(TurretPositions.BallShootSpeedOff))));

		return AutomatedAction.fromAction(new SeriesAction(actionArrayList), Constants.kActionTimeoutS, Turret.getInstance(), BallIntakeArm.getInstance());
	}

	public static AutomatedAction fullyAutomatedTest() {
		ArrayList<Action> actionArrayList = new ArrayList<>();

		actionArrayList.add(new SetBallArmRotationAction(BallIntakeArmPositions.Up, 5));
		actionArrayList.add(new SetBallArmRotationAction(BallIntakeArmPositions.Down, 5));
		actionArrayList.add(pickupHatchFeederStation());
		actionArrayList.add(new SetElevatorHeightAction(ElevatorPositions.RocketBallLow));
		actionArrayList.add(new SetTurretPositionAction(TurretPositions.Left90));
		actionArrayList.add(new SetElevatorHeightAction(ElevatorPositions.RocketHatchMed));
		actionArrayList.add(new SetTurretPositionAction(TurretPositions.Right90));
		actionArrayList.add(shootBall());
		actionArrayList.add(intakeBallOn((t) -> false));
		actionArrayList.add(intakeBallOff());
		actionArrayList.add(new OpenLoopDrive(0.2, 0.2, 2));
		actionArrayList.add(new SetBallArmRotationAction(BallIntakeArmPositions.Up, 5));
		actionArrayList.add(new WaitAction(2));

		return AutomatedAction.fromAction(new SeriesAction(actionArrayList), 100, Turret.getInstance(), BallIntakeArm.getInstance(), Drive.getInstance(), Elevator.getInstance());
	}

}