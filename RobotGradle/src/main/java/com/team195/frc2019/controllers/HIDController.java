package com.team195.frc2019.controllers;

import com.team195.frc2019.Constants;
import com.team195.frc2019.auto.AutoModeExecutor;
import com.team195.frc2019.auto.actions.*;
import com.team195.frc2019.auto.actions.climb.SetBallIntakeArmRotationOpenLoopAction;
import com.team195.frc2019.auto.autonomy.AutomatedAction;
import com.team195.frc2019.auto.autonomy.AutomatedActions;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.subsystems.*;
import com.team195.frc2019.subsystems.positions.BallIntakeArmPositions;
import com.team195.frc2019.subsystems.positions.ElevatorPositions;
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
							double turn;
							if (VisionTracker.getInstance().isVisionEnabled()) {
								turn = Math.max(Math.min(VisionTracker.getInstance().getTargetHorizAngleDev() * 0.01, 1), -1);
							}
							else {
								if (Math.abs(throttle) > 0.5) {
									// turn = driveJoystick.getSmoothedAxis(4, 0.08, 2) * scalingFactor;
									turn = driveJoystick.getNormalizedAxis(4, 0.08) * scalingFactor;
								} else {
									turn = driveJoystick.getNormalizedAxis(4, 0.08) * 0.35;
								}
							}
							// double turn = driveJoystick.getNormalizedAxis(4, 0.08) * scalingFactor * 0.65;
							// turn *= 0.75;

							// boolean quickTurn = driveJoystick.getRawButton(5);

							if (Elevator.getInstance().getPosition() > Constants.kElevatorLowSensitivityThreshold) {
								throttle *= Constants.kLowSensitivityFactor;
								turn *= Constants.kLowSensitivityFactor;
							}

//							mDrive.setOpenLoop(mCheesyDriveHelper.cheesyDrive(throttle, turn, quickTurn, mDrive.isHighGear()));

							if (buttonBox2.getRisingEdgeButton(7)) {
								BallIntakeArm.getInstance().configureClimbCurrentLimit();
//								Drive.getInstance().configureClimbCurrentLimit();
								TeleopActionRunner.runAction(AutomatedActions.climbOpen((t) -> buttonBox2.getRawButton(7), (t) -> -driveJoystick.getNormalizedAxis(1, 0.1)/3.0, (t) -> -driveJoystick.getNormalizedAxis(5, 0.1)));
							}
							else if (mDrive.getDriveControlState() == Drive.DriveControlState.OPEN_LOOP) {
								mDrive.setOpenLoop(new DriveSignal(Math.max(Math.min(throttle + turn, 1), -1), Math.max(Math.min(throttle - turn, 1), -1)));
							}

							if (driveJoystick.getRisingEdgeButton(1)) {
								TeleopActionRunner.runAction(AutomatedActions.enableHatchVision((t) -> driveJoystick.getRawButton(1)));
							}
							else if (driveJoystick.getRisingEdgeTrigger(2, Constants.kJoystickTriggerThreshold)) {
								TeleopActionRunner.runAction(AutomatedActions.rollerHatchFloorIntake((t) -> driveJoystick.getRawAxis(2) > Constants.kJoystickTriggerThreshold));
							} else if (driveJoystick.getRisingEdgeTrigger(3, Constants.kJoystickTriggerThreshold)) {
								TeleopActionRunner.runAction(AutomatedActions.intakeBallOn((t) -> driveJoystick.getRawAxis(3) > Constants.kJoystickTriggerThreshold));
							}

							if (buttonBox1.getRisingEdgeButton(1)) {
								TeleopActionRunner.runAction(AutomatedActions.intakeBallOn((t) -> buttonBox1.getRawButton(1)));
							}
							else if (buttonBox1.getRisingEdgeButton(2)) {
								TeleopActionRunner.runAction(AutomatedActions.elevatorSet(ElevatorPositions.CargoBall));
							}
							else if (buttonBox1.getRisingEdgeButton(3)) {
								TeleopActionRunner.runAction(AutomatedActions.elevatorSet(ElevatorPositions.RocketBallLow));
							}
							else if (buttonBox1.getRisingEdgeButton(4)) {
								TeleopActionRunner.runAction(AutomatedActions.elevatorSet(ElevatorPositions.RocketBallMed));
							}
							else if (buttonBox1.getRisingEdgeButton(5)) {
								TeleopActionRunner.runAction(AutomatedActions.elevatorSet(ElevatorPositions.RocketBallHigh));
							}
							else if (buttonBox1.getRisingEdgeButton(7)) {
								TeleopActionRunner.runAction(AutomatedActions.pickupHatchFeederStation());
							}
							else if (buttonBox1.getRisingEdgeButton(8)) {
								TeleopActionRunner.runAction(AutomatedActions.elevatorSet(ElevatorPositions.CargoHatch));
							}
							else if (buttonBox1.getRisingEdgeButton(9)) {
								TeleopActionRunner.runAction(AutomatedActions.elevatorSet(ElevatorPositions.RocketHatchLow));
							}
							else if (buttonBox1.getRisingEdgeButton(10)) {
								TeleopActionRunner.runAction(AutomatedActions.elevatorSet(ElevatorPositions.RocketHatchMed));
							}
							else if (buttonBox1.getRisingEdgeButton(11)) {
								TeleopActionRunner.runAction(AutomatedActions.elevatorSet(ElevatorPositions.RocketHatchHigh));
							}
							else if (buttonBox1.getRisingEdgeButton(12)) {
								TeleopActionRunner.runAction(AutomatedActions.rollerHatchFloorIntake((t) -> buttonBox1.getRawButton(12)));
							}
							else if (buttonBox1.getRisingEdgeButton(13)) {
								BallIntakeArm.getInstance().setSensorsForReset();
								TeleopActionRunner.runAction(AutomatedActions.ballArmSet(BallIntakeArmPositions.Up));
							}
							else if (buttonBox1.getRisingEdgeButton(14)) {
								TeleopActionRunner.runAction(AutomatedActions.ballArmSet(BallIntakeArmPositions.Down));
							}


							if (buttonBox2.getRisingEdgeButton(7)) {
								TeleopActionRunner.runAction(AutomatedActions.prepareClimb());
							}
							else if (buttonBox2.getRisingEdgeButton(8)) {

							}
							else if (buttonBox2.getRisingEdgeButton(9)) {
							    TeleopActionRunner.runAction(AutomatedActions.ballOuttake((t) -> buttonBox2.getRawButton(9)));
							}
							else if (buttonBox2.getRisingEdgeButton(10)) {
								TeleopActionRunner.runAction(new AutomatedAction(new SetBeakAction(false), 1));
							}
							else if (buttonBox2.getRisingEdgeButton(11)) {

							}
							else if (buttonBox2.getRisingEdgeButton(13)) {
								TeleopActionRunner.runAction(AutomatedActions.climbAutomated((t) -> buttonBox2.getRawButton(13)));
							}
							else if (buttonBox2.getRisingEdgeButton(14)) {
								//Flash LEDs
								LEDController.getInstance().setRequestedState(LEDController.LEDState.BLINK);
							}

							if (armControlJoystick.getRisingEdgeButton(2)) {
								//Manual turret spin and twist Z axis 2
								//TeleopActionRunner.runAction(AutomatedActions.setTurretOpenLoop((t) -> armControlJoystick.getRawButton(2),
								//													(t) -> armControlJoystick.getRawAxis(2)));
							}

							if (armControlJoystick.getRisingEdgeButton(1)) {
								//Flash LEDs to signal Human Player
								LEDController.getInstance().setLEDColor(Constants.kRequestGamePieceColor);
								LEDController.getInstance().setRequestedState(LEDController.LEDState.BLINK);
							}
							else if (armControlJoystick.getRisingEdgeButton(3)) {
								//Ball Outtake turret and arm
								TeleopActionRunner.runAction(AutomatedActions.shootBall());
							}
							else if (armControlJoystick.getRisingEdgeButton(4)) {
								TeleopActionRunner.runAction(AutomatedActions.placeHatch());
							}
							else if (armControlJoystick.getRisingEdgeButton(5)) {
//								TeleopActionRunner.runAction(new AutomatedAction(new SetHatchPushAction(false), 1));
							}
							else if (armControlJoystick.getRisingEdgeButton(6)) {
//								TeleopActionRunner.runAction(new AutomatedAction(new SetHatchPushAction(true), 1));
								TeleopActionRunner.runAction(AutomatedActions.enableHatchVision((t) -> armControlJoystick.getRawButton(6)));
							}
							else if (armControlJoystick.getRisingEdgeButton(7)) {
								//Rehome Elevator
								TeleopActionRunner.runAction(AutomatedActions.homeElevator());
							}
							else if (armControlJoystick.getRisingEdgeButton(9)) {
								//Rehome Arm
							}
							else if (armControlJoystick.getRisingEdgeButton(11)) {
								//Rehome turret
								// Turret.getInstance().zeroSensors();
								// Turret.getInstance().setTurretControlMode(Turret.TurretControlMode.POSITION);
								TeleopActionRunner.runAction(AutomatedActions.reverseHatchPickup());
							}
							else if (armControlJoystick.getRisingEdgeButton(12)) {
								TeleopActionRunner.runAction(AutomatedActions.lowerIntakeAndResetTurret());
							}

							switch (armControlJoystick.getPOV()) {
								case 0:
									TeleopActionRunner.runAction(AutomatedActions.setTurretPosition(TurretPositions.Home));
									break;
								case 90:
									TeleopActionRunner.runAction(AutomatedActions.setTurretPosition(TurretPositions.Left90));
									break;
								case 180:
									TeleopActionRunner.runAction(AutomatedActions.setTurretPosition(TurretPositions.Back180));
									break;
								case 270:
									TeleopActionRunner.runAction(AutomatedActions.setTurretPosition(TurretPositions.Right90));
									break;
								default:
									break;
							}
						}
					} catch (Exception ex) {
						ConsoleReporter.report(ex);
					}
					catch (Throwable t) {
						ConsoleReporter.report(t);
						CrashTracker.logThrowableCrash(t);
//						throw t;
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
