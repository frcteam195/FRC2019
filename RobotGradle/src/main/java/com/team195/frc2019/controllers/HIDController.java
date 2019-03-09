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
import com.team254.lib.util.DriveSignal;

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
							double scalingFactor = driveJoystick.getRawButton(6) ? 1 : 1;

//							double throttle = -driveJoystick.getRawAxis(1) * scalingFactor;
//							double turn = driveJoystick.getRawAxis(4) * scalingFactor;

							double throttle = -driveJoystick.getNormalizedAxis(1, 0.08) * scalingFactor;
							double turn = driveJoystick.getNormalizedAxis(4, 0.08) * scalingFactor;

							boolean quickTurn = driveJoystick.getRawButton(5);

							if (Elevator.getInstance().getPosition() > Constants.kElevatorLowSensitivityThreshold) {
								throttle *= Constants.kLowSensitivityFactor;
								turn *= Constants.kLowSensitivityFactor;
							}



//							mDrive.setOpenLoop(mCheesyDriveHelper.cheesyDrive(throttle, turn, quickTurn, mDrive.isHighGear()));

							mDrive.setOpenLoop(new DriveSignal(Math.max(Math.min(throttle + turn, 1), -1), Math.max(Math.min(throttle - turn, 1), -1)));

							if (driveJoystick.getRisingEdgeTrigger(2, Constants.kJoystickTriggerThreshold)) {
								(new TeleopActionRunner(AutomatedActions.rollerHatchFloorIntake((t) -> driveJoystick.getRawAxis(2) > Constants.kJoystickTriggerThreshold))).runAction();
							} else if (driveJoystick.getRisingEdgeTrigger(3, Constants.kJoystickTriggerThreshold)) {
								(new TeleopActionRunner(AutomatedActions.intakeBallOn((t) -> driveJoystick.getRawAxis(3) > Constants.kJoystickTriggerThreshold))).runAction();
							}

							if (buttonBox1.getRisingEdgeButton(1)) {
								(new TeleopActionRunner(AutomatedActions.intakeBallOn((t) -> buttonBox1.getRawButton(1)))).runAction();
							}
							else if (buttonBox1.getRisingEdgeButton(2)) {
								(new TeleopActionRunner(new SetElevatorHeightAction(ElevatorPositions.CargoBall))).runAction();
							}
							else if (buttonBox1.getRisingEdgeButton(3)) {
								(new TeleopActionRunner(new SetElevatorHeightAction(ElevatorPositions.RocketBallLow))).runAction();
							}
							else if (buttonBox1.getRisingEdgeButton(4)) {
								(new TeleopActionRunner(new SetElevatorHeightAction(ElevatorPositions.RocketBallMed))).runAction();
							}
							else if (buttonBox1.getRisingEdgeButton(5)) {
								(new TeleopActionRunner(new SetElevatorHeightAction(ElevatorPositions.RocketBallHigh))).runAction();
							}
							else if (buttonBox1.getRisingEdgeButton(7)) {
								(new TeleopActionRunner(AutomatedActions.pickupHatchFeederStation())).runAction();
							}
							else if (buttonBox1.getRisingEdgeButton(8)) {
								(new TeleopActionRunner(new SetElevatorHeightAction(ElevatorPositions.CargoHatch))).runAction();
							}
							else if (buttonBox1.getRisingEdgeButton(9)) {
								(new TeleopActionRunner(new SetElevatorHeightAction(ElevatorPositions.RocketHatchLow))).runAction();
							}
							else if (buttonBox1.getRisingEdgeButton(10)) {
								(new TeleopActionRunner(new SetElevatorHeightAction(ElevatorPositions.RocketHatchMed))).runAction();
							}
							else if (buttonBox1.getRisingEdgeButton(11)) {
								(new TeleopActionRunner(new SetElevatorHeightAction(ElevatorPositions.RocketHatchHigh))).runAction();
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
							else if (buttonBox2.getRisingEdgeButton(9)) {
								(new TeleopActionRunner(AutomatedActions.shootBall())).runAction();
							}
							else if (buttonBox2.getRisingEdgeButton(10)) {
								(new TeleopActionRunner(AutomatedActions.placeHatch())).runAction();
							}
							else if (buttonBox2.getRisingEdgeButton(14)) {
								//Flash LEDs
								LEDController.getInstance().setRequestedState(LEDController.LEDState.BLINK);
							}

							if (armControlJoystick.getRisingEdgeButton(2)) {
								//Manual turret spin and twist Z axis 2
								(new TeleopActionRunner(new SetTurretOpenLoopAction((t) -> armControlJoystick.getRawButton(2),
																					(t) -> armControlJoystick.getRawAxis(2)), 100)).runAction();
							}

							if (armControlJoystick.getRisingEdgeButton(1)) {
								//Flash LEDs to signal Human Player
								LEDController.getInstance().setRequestedState(LEDController.LEDState.BLINK);
							}
							else if (armControlJoystick.getRisingEdgeButton(3)) {
								//Ball Outtake turret and arm
								(new TeleopActionRunner(AutomatedActions.ballOuttake((t) -> armControlJoystick.getRawButton(3)))).runAction();
							}
							else if (armControlJoystick.getRisingEdgeButton(4)) {
								//Close Beak and Hatch roller outtake hold
								(new TeleopActionRunner(new SetBeakAction(false))).runAction();
//								(new TeleopActionRunner(AutomatedActions.rollerHatchFloorIntake((t) -> armControlJoystick.getRawButton(4)))).runAction();
							}
							else if (armControlJoystick.getRisingEdgeButton(5)) {
								(new TeleopActionRunner(new SetHatchPushAction(false))).runAction();
							}
							else if (armControlJoystick.getRisingEdgeButton(6)) {
								(new TeleopActionRunner(new SetHatchPushAction(true))).runAction();
							}
							else if (armControlJoystick.getRisingEdgeButton(7)) {
								//Rehome Elevator
							}
							else if (armControlJoystick.getRisingEdgeButton(9)) {
								//Rehome Arm
							}
							else if (armControlJoystick.getRisingEdgeButton(11)) {
								//Rehome turret
								Turret.getInstance().zeroSensors();
								Turret.getInstance().setTurretControlMode(Turret.TurretControlMode.POSITION);
							}

							switch (armControlJoystick.getPOV()) {
								case 0:
									(new TeleopActionRunner(new SetTurretPositionAction(TurretPositions.Home))).runAction();
									break;
								case 90:
									(new TeleopActionRunner(new SetTurretPositionAction(TurretPositions.Left90))).runAction();
									break;
								case 180:
									(new TeleopActionRunner(new SetTurretPositionAction(TurretPositions.Back180))).runAction();
									break;
								case 270:
									(new TeleopActionRunner(new SetTurretPositionAction(TurretPositions.Right90))).runAction();
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
