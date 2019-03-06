package com.team195.frc2019.controllers;

import com.team195.frc2019.Constants;
import com.team195.frc2019.auto.AutoModeExecutor;
import com.team195.frc2019.auto.actions.*;
import com.team195.frc2019.subsystems.*;
import com.team195.frc2019.subsystems.positions.BallIntakeArmPositions;
import com.team195.frc2019.subsystems.positions.ElevatorPositions;
import com.team195.frc2019.subsystems.positions.HatchArmPositions;
import com.team195.frc2019.subsystems.positions.TurretPositions;
import com.team195.lib.drivers.dashjoy.CKDashJoystick;
import com.team195.lib.util.TeleopActionRunner;
import com.team195.lib.util.ThreadRateControl;
import com.team254.lib.util.CheesyDriveHelper;
import com.team254.lib.util.CrashTracker;

import java.util.function.Function;

public class HIDController {
	private static HIDController mInstance = null;
	public static HIDController getInstance(Function<Void, AutoModeExecutor> getAutoModeExecutor) {
		if (mInstance == null)
			mInstance = new HIDController(getAutoModeExecutor);

		return mInstance;
	}

	private final CKDashJoystick driveJoystick = Controllers.getInstance().getDriveJoystick();
	private final CKDashJoystick armControlJoystick = Controllers.getInstance().getArmControlJoystick();
	private final CKDashJoystick buttonBox1 = Controllers.getInstance().getButtonBox1();
	private final CKDashJoystick buttonBox2 = Controllers.getInstance().getButtonBox2();

	private CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();
	private Drive mDrive = Drive.getInstance();

	private Thread controlThread = null;
	private boolean runThread = true;

	private final Function<Void, AutoModeExecutor> mGetAutoModeExecutor;

	private static final int HID_RATE_CONTROL = 10;

	private HIDController(Function<Void, AutoModeExecutor> getAutoModeExecutor) {
		mGetAutoModeExecutor = getAutoModeExecutor;
	}

	public void start() {
		if (controlThread == null || !controlThread.isAlive()) {
			setRunState(true);
			controlThread = new Thread(() -> {
				ThreadRateControl threadRateControl = new ThreadRateControl();
				threadRateControl.start();
				while (runThread) {
					try {
						AutoModeExecutor autoModeExecutor = mGetAutoModeExecutor.apply(null);
						if (autoModeExecutor != null && autoModeExecutor.isRunning()) {
							if (driveJoystick.isAxisInputActive())
								autoModeExecutor.stop();
						} else {
							//User Control Interface code here

							double throttle = -driveJoystick.getNormalizedAxis(1, 0.04)/2.0;
							double turn = driveJoystick.getNormalizedAxis(4, 0.04)/2.0;
							boolean quickTurn = driveJoystick.getRawButton(5);

							if (Elevator.getInstance().getPosition() > Constants.kElevatorLowSensitivityThreshold) {
								throttle *= Constants.kLowSensitivityFactor;
								turn *= Constants.kLowSensitivityFactor;
							}
							mDrive.setOpenLoop(mCheesyDriveHelper.cheesyDrive(throttle, turn, quickTurn, mDrive.isHighGear()));

							if (driveJoystick.getRawAxis(2) > Constants.kJoystickTriggerThreshold) {
								//Left Trigger - Ground hatch intake

							} else if (driveJoystick.getRawAxis(3) > Constants.kJoystickTriggerThreshold) {
								//Right Trigger - Ball Intake

							}


							if (buttonBox1.getRisingEdgeButton(1)) {
								(new TeleopActionRunner(AutomatedActions.intakeBallOn((t) -> buttonBox1.getRawButton(1)))).runAction();
							}
							else if (buttonBox1.getRisingEdgeButton(2)) {
								(new TeleopActionRunner(AutomatedActions.shootBall(ElevatorPositions.CargoBall, (t) -> buttonBox1.getRawButton(2)))).runAction();
							}
							else if (buttonBox1.getRisingEdgeButton(3)) {
								(new TeleopActionRunner(AutomatedActions.shootBall(ElevatorPositions.RocketBallLow, (t) -> buttonBox1.getRawButton(3)))).runAction();
							}
							else if (buttonBox1.getRisingEdgeButton(4)) {
								(new TeleopActionRunner(AutomatedActions.shootBall(ElevatorPositions.RocketBallMed, (t) -> buttonBox1.getRawButton(4)))).runAction();
							}
							else if (buttonBox1.getRisingEdgeButton(5)) {
								(new TeleopActionRunner(AutomatedActions.shootBall(ElevatorPositions.RocketBallHigh, (t) -> buttonBox1.getRawButton(5)))).runAction();
							}
							else if (buttonBox1.getRisingEdgeButton(7)) {
								(new TeleopActionRunner(AutomatedActions.pickupHatchFeederStation())).runAction();
							}
							else if (buttonBox1.getRisingEdgeButton(8)) {
								(new TeleopActionRunner(AutomatedActions.placeHatch(ElevatorPositions.CargoHatch, (t) -> buttonBox1.getRawButton(8)))).runAction();
							}
							else if (buttonBox1.getRisingEdgeButton(9)) {
								(new TeleopActionRunner(AutomatedActions.placeHatch(ElevatorPositions.RocketHatchLow, (t) -> buttonBox1.getRawButton(9)))).runAction();
							}
							else if (buttonBox1.getRisingEdgeButton(10)) {
								(new TeleopActionRunner(AutomatedActions.placeHatch(ElevatorPositions.RocketHatchMed, (t) -> buttonBox1.getRawButton(10)))).runAction();
							}
							else if (buttonBox1.getRisingEdgeButton(11)) {
								(new TeleopActionRunner(AutomatedActions.placeHatch(ElevatorPositions.RocketHatchHigh, (t) -> buttonBox1.getRawButton(11)))).runAction();
							}
							else if (buttonBox1.getRisingEdgeButton(12)) {
								(new TeleopActionRunner(AutomatedActions.rollerHatchFloorIntake((t) -> buttonBox1.getRawButton(12)))).runAction();
							}
							else if (buttonBox1.getRisingEdgeButton(13)) {
								BallIntakeArm.getInstance().setSensorsForReset();
								(new TeleopActionRunner(new SetBallArmRotationAction(BallIntakeArmPositions.Up))).runAction();
							}
							else if (buttonBox1.getRisingEdgeButton(14)) {
								(new TeleopActionRunner(new SetBallArmRotationAction(BallIntakeArmPositions.Down))).runAction();
							}


							if (buttonBox2.getRisingEdgeButton(6)) {
								(new TeleopActionRunner(new DropBallArmClimbBarAction())).runAction();
							}
							else if (buttonBox2.getRisingEdgeButton(7)) {
								//Climb
							}
							else if (buttonBox2.getRisingEdgeButton(8)) {
								//Reverse Climb
							}
							else if (buttonBox2.getRisingEdgeButton(14)) {
								//Flash LEDs
							}

							if (armControlJoystick.getRisingEdgeButton(2)) {
								//Manual turret spin and twist Z axis 2
								(new TeleopActionRunner(new SetTurretOpenLoopAction((t) -> armControlJoystick.getRawButton(2),
																					(t) -> armControlJoystick.getRawAxis(2)), 100)).runAction();
							}

							if (armControlJoystick.getRisingEdgeButton(1)) {
								//Flash LEDs to signal Human Player
							}
							else if (armControlJoystick.getRisingEdgeButton(3)) {
								//Ball Outtake turret and arm
								(new TeleopActionRunner(AutomatedActions.ballOuttake((t) -> armControlJoystick.getRawButton(3)))).runAction();
							}
							else if (armControlJoystick.getRisingEdgeButton(4)) {
								//Close Beak and Hatch roller outtake hold
								Turret.getInstance().setBeak(true);
								(new TeleopActionRunner(AutomatedActions.rollerHatchFloorIntake((t) -> armControlJoystick.getRawButton(4)))).runAction();
							}
							else if (armControlJoystick.getRisingEdgeButton(5)) {
								Turret.getInstance().setHatchPush(false);
							}
							else if (armControlJoystick.getRisingEdgeButton(6)) {
								Turret.getInstance().setHatchPush(true);
							}
							else if (armControlJoystick.getRisingEdgeButton(7)) {
								//Rehome Elevator
							}
							else if (armControlJoystick.getRisingEdgeButton(9)) {
								//Rehome Arm
							}
							else if (armControlJoystick.getRisingEdgeButton(11)) {
								//Rehome turret
							}

							switch (armControlJoystick.getPOV()) {
								case 0:
									Turret.getInstance().setTurretPosition(TurretPositions.Home);
									break;
								case 90:
									Turret.getInstance().setTurretPosition(TurretPositions.Left90);
									break;
								case 180:
									Turret.getInstance().setTurretPosition(TurretPositions.Back180);
									break;
								case 270:
									Turret.getInstance().setTurretPosition(TurretPositions.Right90);
									break;
								default:
									break;
							}
						}
					} catch (Exception ignored) {

					}
					catch (Throwable t) {
						CrashTracker.logThrowableCrash(t);
						throw t;
					}
					threadRateControl.doRateControl(HID_RATE_CONTROL);
				}
			});
			controlThread.setPriority(Constants.kRobotThreadPriority);
			controlThread.start();
		}
	}

	public void stop() {
		setRunState(false);
		try {
			controlThread.join(100);
		} catch (Exception ignored) {

		} finally {
			controlThread = null;
		}
	}

	private synchronized void setRunState(boolean run) {
		runThread = run;
	}
}
