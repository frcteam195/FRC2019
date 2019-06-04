package com.team195.frc2019.auto.autonomy;

import com.team195.frc2019.auto.actions.climb.*;
import com.team195.frc2019.constants.Constants;
import com.team195.frc2019.constants.AutoConstants;
import com.team195.frc2019.auto.actions.*;
import com.team195.frc2019.subsystems.*;
import com.team195.frc2019.subsystems.positions.BallIntakeArmPositions;
import com.team195.frc2019.subsystems.positions.ElevatorPositions;
import com.team195.frc2019.subsystems.positions.TurretPositions;
import com.team195.lib.drivers.dashjoy.CKDashJoystick;
import com.team195.lib.util.AtomicDouble;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;
import java.util.function.Predicate;

public class AutomatedActions {
	public static final AutomatedAction setBeakOpenAction =  AutomatedAction.fromAction(new SetBeakAction(true), 4);
	public static final AutomatedAction setBeakClosedAction =  AutomatedAction.fromAction(new SetBeakAction(false), 1);

	public static final AutomatedAction setTurretHomeAction = AutomatedAction.fromAction(new SetTurretPositionAction(TurretPositions.Home), Constants.kActionTimeoutS, Turret.getInstance());
	public static final AutomatedAction setTurretLeft90Action = AutomatedAction.fromAction(new SetTurretPositionAction(TurretPositions.Left90), Constants.kActionTimeoutS, Turret.getInstance());
	public static final AutomatedAction setTurretRight90Action = AutomatedAction.fromAction(new SetTurretPositionAction(TurretPositions.Right90), Constants.kActionTimeoutS, Turret.getInstance());
	public static final AutomatedAction setTurretBack180Action = AutomatedAction.fromAction(new SetTurretPositionAction(TurretPositions.Back180), Constants.kActionTimeoutS, Turret.getInstance());

	public static final AutomatedAction lowerIntakeAndResetTurretAction = AutomatedAction.fromAction(new SeriesAction(Arrays.asList(
			new SetHatchPushAction(false),
			new SetBallArmRotationAction(BallIntakeArmPositions.Down),
			new SetTurretPositionAction(TurretPositions.Home)
	)), Constants.kActionTimeoutS, BallIntakeArm.getInstance(), Turret.getInstance());

	public static final AutomatedAction homeElevatorAction = AutomatedAction.fromAction(new SeriesAction(Arrays.asList(
			new SetElevatorOpenLoopAction(-0.25),
			new WaitForElevatorLimitAction(3),
			new SetElevatorHomeAction(),
			new SetLEDAction(Constants.kGotGamePieceColor)
	)), Constants.kActionTimeoutS, Elevator.getInstance());

	private static AtomicReference<Predicate<Void>> shootBallActionButtonGetter = new AtomicReference<>();
	private static final AutomatedAction shootBallAction = AutomatedAction.fromAction(new SeriesAction(Arrays.asList(
			new SetBallShooterOpenLoopAction(TurretPositions.BallShootSpeedNormal),
			new SetBallPushAction(true),
			new WaitAction(AutoConstants.kWaitForBallPush),
			new SetBallPushAction(false),
			new WaitForFallingEdgeButtonAction(shootBallActionButtonGetter, 5),
			new SetBallShooterOpenLoopAction(0)
	)), Constants.kActionTimeoutS, Turret.getInstance());
	public static AutomatedAction shootBall(Predicate<Void> buttonValueGetter) {
		shootBallActionButtonGetter.set(buttonValueGetter);
		return shootBallAction;
	}

	private static AtomicReference<Predicate<Void>> climbMaxButtonGetter = new AtomicReference<>();
	private static final AutomatedAction climbMaxAction = AutomatedAction.fromAction(new SeriesAction(Arrays.asList(
			new ParallelAction(Arrays.asList(new SetElevatorHeightAction(ElevatorPositions.Down),
					new SetTurretPositionAction(TurretPositions.Home))),
			new DropBallArmClimbBarAction(),
			new SetBallArmRotationAction(BallIntakeArmPositions.Down),
			new SetDrivePTOAction(true),
			new ParallelAction(Arrays.asList(new SetBallIntakeAction(BallIntakeArmPositions.RollerIntake),
					new HoldOpenLoopDriveClimbAction(1, 1, climbMaxButtonGetter))),
			new SetClimbRackUpAction(),
			new WaitAction(0.5),
			new SetDrivePTOAction(false),
			new SetDriveRampDownPowerAction(0.4),
			new SetBallIntakeAction(BallIntakeArmPositions.RollerOff)
	)), 300, Drive.getInstance(), BallIntakeArm.getInstance());
	public static AutomatedAction climbMax(Predicate<Void> buttonValueGetter) {
		climbMaxButtonGetter.set(buttonValueGetter);
		BallIntakeArm.getInstance().configureClimbCurrentLimit();
		Drive.getInstance().configureClimbCurrentLimit();
		return climbMaxAction;
	}

	public static final AutomatedAction setPTOToDriveAction = AutomatedAction.fromAction(new SetDrivePTOAction(false), 1);

	private static AtomicReference<Predicate<Void>> ballOuttakeButtonGetter = new AtomicReference<>();
	private static final AutomatedAction ballOuttakeAction = AutomatedAction.fromAction(new SeriesAction(Arrays.asList(
			new ParallelAction(Arrays.asList(new SetBallIntakeAction(BallIntakeArmPositions.RollerOuttake),
					new SetBallShooterOpenLoopAction(TurretPositions.BallShootSpeedNormal))),
			new WaitForFallingEdgeButtonAction(ballOuttakeButtonGetter, 10),
			new ParallelAction(Arrays.asList(new SetBallIntakeAction(BallIntakeArmPositions.RollerOff),
					new SetBallShooterOpenLoopAction(TurretPositions.BallShootSpeedOff)))
	)), Constants.kActionTimeoutS, Turret.getInstance(), BallIntakeArm.getInstance());
	public static AutomatedAction ballOuttake(Predicate<Void> buttonValueGetter) {
		ballOuttakeButtonGetter.set(buttonValueGetter);
		return ballOuttakeAction;
	}

	public static final AutomatedAction ballArmSetUpAction = AutomatedAction.fromAction(new SeriesAction(Arrays.asList(
			new SetHatchPushAction(false),
			new SetBallArmRotationAction(BallIntakeArmPositions.Up, 5)
	)), Constants.kActionTimeoutS, BallIntakeArm.getInstance());

	public static final AutomatedAction ballArmSetDownAction = AutomatedAction.fromAction(new SeriesAction(Arrays.asList(
			new SetHatchPushAction(false),
			new SetBallArmRotationAction(BallIntakeArmPositions.Down, 5)
	)), Constants.kActionTimeoutS, BallIntakeArm.getInstance());

	public static final AutomatedAction dropBallArmClimbBarAction = AutomatedAction.fromAction(new DropBallArmClimbBarAction(), Constants.kActionTimeoutS);

	private static final AtomicDouble elevatorSetpoint = new AtomicDouble();
	private static final AutomatedAction elevatorSetAction = AutomatedAction.fromAction(new SeriesAction(Arrays.asList(
			new SetHatchPushAction(false),
			new SetBallArmRotationAction(BallIntakeArmPositions.Down),
			new SetElevatorHeightAction(elevatorSetpoint)
	)), Constants.kActionTimeoutS, Turret.getInstance(), Elevator.getInstance());
	public static AutomatedAction elevatorSet(double elevatorHeight) {
		elevatorSetpoint.set(elevatorHeight);
		return elevatorSetAction;
	}

	private static final AutomatedAction unfoldArmIsUp = AutomatedAction.fromAction(new SeriesAction(Arrays.asList(
			new SetHatchPushAction(false),
			new ParallelAction(Arrays.asList(new SetBallArmRotationAction(BallIntakeArmPositions.Down),
					new SetElevatorHeightAction(ElevatorPositions.RocketHatchLow)))
	)), Constants.kActionTimeoutS, BallIntakeArm.getInstance(), Elevator.getInstance());
	private static final AutomatedAction unfoldArmIsDown = AutomatedAction.fromAction(new SeriesAction(Arrays.asList(
			new SetHatchPushAction(false),
			new SetElevatorHeightAction(ElevatorPositions.RocketHatchLow)
	)), Constants.kActionTimeoutS, BallIntakeArm.getInstance(), Elevator.getInstance());
	public static AutomatedAction unfold() {
		if (BallIntakeArm.getInstance().isArmUp())
			return unfoldArmIsUp;
		else
			return unfoldArmIsDown;
	}

	private static AtomicReference<Function<Void, CKDashJoystick>> pickupHatchFeederStationJoystickGetter = new AtomicReference<>();
	private static final AutomatedAction pickupHatchFeederStationAction = AutomatedAction.fromAction(new SeriesAction(Arrays.asList(
			new SetHatchPushAction(false),
			new SetBallArmRotationAction(BallIntakeArmPositions.Down),
			new ParallelAction(Arrays.asList(new SetElevatorHeightAction(ElevatorPositions.HatchPickupStation),
					new SetBeakAction(false),
					new SeriesAction(Arrays.asList(new WaitForElevatorGreaterThanPositionAction(ElevatorPositions.CollisionThresholdTurret, 1),
							new SetTurretPositionAction(TurretPositions.Home))))),
			new SetBeakSenseAction(true),
			new SetHatchPushAction(true),
			new WaitForHatchOrTimeoutAction(),
			new SetElevatorHeightAction(ElevatorPositions.HatchPickupStationLift),
			new WaitAction(0.2),
			new SetHatchPushAction(false),
			new WaitAction(0.2),
			new SetElevatorHeightAction(ElevatorPositions.RocketHatchLow),
			new SetRumbleIfHatchAction(pickupHatchFeederStationJoystickGetter, 1)
	)), Constants.kActionTimeoutS, Elevator.getInstance(), Turret.getInstance());
	public static AutomatedAction pickupHatchFeederStation(Function<Void, CKDashJoystick> driverJoystickGetter) {
		pickupHatchFeederStationJoystickGetter.set(driverJoystickGetter);
		return pickupHatchFeederStationAction;
	}

	public static final AutomatedAction placeHatchAction = AutomatedAction.fromAction(new SeriesAction(Arrays.asList(
			new SetBeakSenseAction(false),
			new SetHatchPushAction(true),
			new WaitAction(AutoConstants.kWaitForHatchPush),
			new SetBeakForcedClosedAction(),
			new WaitAction(AutoConstants.kWaitForHatchPush),
			new SetHatchPushAction(false),
			new WaitAction(2),
			new SetBeakAction(false),
			new SetBeakSenseAction(true)
	)), Constants.kActionTimeoutS, Turret.getInstance());

	public static final AutomatedAction placeHatchAutoAction = AutomatedAction.fromAction(new SeriesAction(Arrays.asList(
			new SetBeakSenseAction(false),
			new SetHatchPushAction(true),
			new WaitAction(AutoConstants.kWaitForHatchPush),
			new SetBeakForcedClosedAction(),
			new WaitAction(AutoConstants.kWaitForHatchPush),
			new SetHatchPushAction(false),
			new WaitAction(0.1),
			new SetBeakAction(false),
			new SetBeakSenseAction(true)
	)), Constants.kActionTimeoutS, Turret.getInstance());

	private static AtomicReference<Predicate<Void>> intakeBallOnButtonGetter = new AtomicReference<>();
	private static final AutomatedAction intakeBallOnAction = AutomatedAction.fromAction(new SeriesAction(Arrays.asList(
			new SetHatchPushAction(false),
			new SetBallArmRotationAction(BallIntakeArmPositions.Down),
			new ParallelAction(Arrays.asList(new SetTurretPositionAction(TurretPositions.Home),
					new SeriesAction(Arrays.asList(new WaitForTurretLessThanRotationAction(TurretPositions.Right90, 1),
							new SetElevatorHeightAction(ElevatorPositions.Down))))),
			new ParallelAction(Arrays.asList(new SetBallShooterOpenLoopAction(TurretPositions.BallShootSpeedIntake),
					new SetBallIntakeAction(BallIntakeArmPositions.RollerIntake))),
			new WaitForFallingEdgeButtonAction(intakeBallOnButtonGetter, 15),
			new ParallelAction(Arrays.asList(new SetBallShooterOpenLoopAction(TurretPositions.BallShootSpeedOff),
					new SetBallIntakeAction(BallIntakeArmPositions.RollerOff)))
	)), Constants.kActionTimeoutS, Elevator.getInstance(), BallIntakeArm.getInstance(), Turret.getInstance());
	public static AutomatedAction intakeBallOn(Predicate<Void> buttonValueGetter) {
		intakeBallOnButtonGetter.set(buttonValueGetter);
		return intakeBallOnAction;
	}

	private static AtomicReference<Predicate<Void>> reverseIntakeBallButtonGetter = new AtomicReference<>();
	private static final AutomatedAction reverseIntakeBallAction = AutomatedAction.fromAction(new SeriesAction(Arrays.asList(
			new SetHatchPushAction(false),
			new SetBallArmRotationAction(BallIntakeArmPositions.Down),
			new ParallelAction(new SetElevatorHeightAction(ElevatorPositions.RocketHatchMed),
					new SeriesAction(new WaitForElevatorGreaterThanPositionAction(ElevatorPositions.CollisionThresholdTurret, 1),
							new SetTurretPositionAction(TurretPositions.Back180))),
			new SetBallShooterOpenLoopAction(TurretPositions.BallShootSpeedIntake),
			new WaitForFallingEdgeButtonAction(reverseIntakeBallButtonGetter, 15),
			new ParallelAction(Arrays.asList(new SetBallShooterOpenLoopAction(TurretPositions.BallShootSpeedOff),
					new SetBallIntakeAction(BallIntakeArmPositions.RollerOff))),
			new ParallelAction(Arrays.asList(new SetTurretPositionAction(TurretPositions.Home),
					new SeriesAction(Arrays.asList(new WaitForTurretLessThanRotationAction(TurretPositions.Right90, 1),
							new SetElevatorHeightAction(ElevatorPositions.Down)))))
			)), Constants.kActionTimeoutS, Elevator.getInstance(), BallIntakeArm.getInstance(), Turret.getInstance());
	public static AutomatedAction reverseIntakeBall(Predicate<Void> buttonValueGetter) {
		reverseIntakeBallButtonGetter.set(buttonValueGetter);
		return reverseIntakeBallAction;
	}










	public static AutomatedAction climbOpen(Function<Void, Boolean> buttonGetterMethod, Function<Void, Double> axisFrontGetterMethod, Function<Void, Double> axisBackGetterMethod) {
		ArrayList<Action> actionArrayList = new ArrayList<>();

		actionArrayList.add(new SetDrivePTOAction(true));
		actionArrayList.add(new SetOpenLoopDriveAction(buttonGetterMethod, axisBackGetterMethod, axisFrontGetterMethod));

		return AutomatedAction.fromAction(new SeriesAction(actionArrayList), 300, Drive.getInstance());
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

	public static AutomatedAction fullyAutomatedTest() {
		ArrayList<Action> actionArrayList = new ArrayList<>();

		actionArrayList.add(new SetBallArmRotationAction(BallIntakeArmPositions.Up, 5));
		actionArrayList.add(new SetBallArmRotationAction(BallIntakeArmPositions.Down, 5));
		actionArrayList.add(pickupHatchFeederStation(null));
		actionArrayList.add(new SetElevatorHeightAction(ElevatorPositions.RocketBallLow));
		actionArrayList.add(new SetTurretPositionAction(TurretPositions.Left90));
		actionArrayList.add(new SetElevatorHeightAction(ElevatorPositions.RocketHatchMed));
		actionArrayList.add(new SetTurretPositionAction(TurretPositions.Right90));
		actionArrayList.add(shootBallAction);
		actionArrayList.add(intakeBallOn((t) -> false));
		actionArrayList.add(new OpenLoopDrive(0.2, 0.2, 2));
		actionArrayList.add(new SetBallArmRotationAction(BallIntakeArmPositions.Up, 5));
		actionArrayList.add(new WaitAction(2));

		return AutomatedAction.fromAction(new SeriesAction(actionArrayList), 100, Turret.getInstance(), BallIntakeArm.getInstance(), Drive.getInstance(), Elevator.getInstance());
	}

}