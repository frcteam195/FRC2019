package com.team195.frc2019.controllers;

import com.team195.frc2019.Robot;
import com.team195.frc2019.constants.CalConstants;
import com.team195.frc2019.constants.Constants;
import com.team195.frc2019.auto.AutoModeExecutor;
import com.team195.frc2019.auto.actions.*;
import com.team195.frc2019.auto.autonomy.AutomatedAction;
import com.team195.frc2019.auto.autonomy.AutomatedActions;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.reporters.MessageLevel;
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
	public static HIDController getInstance() {
		if (mInstance == null)
			mInstance = new HIDController();

		return mInstance;
	}

	private static final boolean USE_CHEESY_DRIVE = true;

	private final CKDashJoystick driveJoystick = Controllers.getInstance().getDriveJoystick();
	private final CKDashJoystick armControlJoystick = Controllers.getInstance().getArmControlJoystick();
	private final CKDashJoystick buttonBox1 = Controllers.getInstance().getButtonBox1();
	private final CKDashJoystick buttonBox2 = Controllers.getInstance().getButtonBox2();

	private CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();
	private Drive mDrive = Drive.getInstance();

	private Thread controlThread = null;
	private boolean runThread = true;
	private boolean stoppedAuto = false;

	private static final int HID_RATE_CONTROL = 10;

	private HIDController() {

	}

	public void start() {
		if (controlThread == null || !controlThread.isAlive()) {
			setRunState(true);
			controlThread = new Thread(() -> {
				Thread.currentThread().setName("HIDController");
				ThreadRateControl threadRateControl = new ThreadRateControl();
				threadRateControl.start();
				while (runThread) {
					try {
						if (Infrastructure.getInstance().isDuringAuto()) {
							AutoModeExecutor autoModeExecutor = Robot.mAutoModeExecutor;
							if (driveJoystick.isAxisInputActive()) {
								ConsoleReporter.report("Stopping auto");
								if (autoModeExecutor != null)
									autoModeExecutor.stop();
								stoppedAuto = true;
							}
						} else {
							//User Control Interface code here

							if (!USE_CHEESY_DRIVE) {
								double scalingFactor = driveJoystick.getRawButton(6) ? 1 : 1;

								double throttle = -driveJoystick.getSmoothedAxis(1, Constants.kJoystickDeadband, 2) * scalingFactor;
								double turn = driveJoystick.getNormalizedAxis(4, Constants.kJoystickDeadband) * scalingFactor * 0.5;
								if (VisionTracker.getInstance().isVisionEnabled()) {
									if (Turret.getInstance().getSetpoint() == TurretPositions.Right90) {
										if (VisionTracker.getInstance().isTargetFound())
											throttle = -Math.max(Math.min(VisionTracker.getInstance().getTargetHorizAngleDev() * 0.01, 1), -1);
									} else if (Turret.getInstance().getSetpoint() == TurretPositions.Left90) {
										if (VisionTracker.getInstance().isTargetFound())
											throttle = Math.max(Math.min(VisionTracker.getInstance().getTargetHorizAngleDev() * 0.01, 1), -1);
									} else {
										if (VisionTracker.getInstance().isTargetFound())
											turn = Math.max(Math.min(VisionTracker.getInstance().getTargetHorizAngleDev() * 0.007, 1), -1);
									}
								}

								if (Elevator.getInstance().getPosition() > CalConstants.kElevatorLowSensitivityThreshold) {
									throttle *= CalConstants.kLowSensitivityFactor;
									turn *= CalConstants.kLowSensitivityFactor;
								}

//							    if (driveJoystick.getRisingEdgeButton(5)) {
//							    	mDrive.setDriveControlState(Drive.DriveControlState.OPEN_LOOP);
//							    }
//							    else if (driveJoystick.getRisingEdgeButton(6)) {
//							    	TeleopActionRunner.runAction(AutomatedAction.fromAction(
//							    			new TurnDriveLooseAngleAction(180, 2), 6, Drive.getInstance()));
//							    }

								if (mDrive.getDriveControlState() == Drive.DriveControlState.OPEN_LOOP) {
									mDrive.setBrakeMode(driveJoystick.getRawButton(5) || driveJoystick.getRawButton(6) || VisionTracker.getInstance().isVisionEnabled());
									mDrive.setOpenLoop(new DriveSignal(Math.max(Math.min(throttle + turn, 1), -1), Math.max(Math.min(throttle - turn, 1), -1)));
								}
							} else {
								double throttle = -driveJoystick.getNormalizedAxis(1, Constants.kJoystickDeadband);
								double turn = driveJoystick.getNormalizedAxis(4, Constants.kJoystickDeadband) * 0.75;

								if (VisionTracker.getInstance().isVisionEnabled()) {
									if (Turret.getInstance().getSetpoint() == TurretPositions.Right90) {
										if (VisionTracker.getInstance().isTargetFound())
											throttle = -Math.max(Math.min(VisionTracker.getInstance().getTargetHorizAngleDev() * 0.01, 1), -1);
									} else if (Turret.getInstance().getSetpoint() == TurretPositions.Left90) {
										if (VisionTracker.getInstance().isTargetFound())
											throttle = Math.max(Math.min(VisionTracker.getInstance().getTargetHorizAngleDev() * 0.01, 1), -1);
									} else {
										if (VisionTracker.getInstance().isTargetFound())
											turn = Math.max(Math.min(VisionTracker.getInstance().getTargetHorizAngleDev() * 0.007, 0.1), -0.1);
									}
								}

								if (Elevator.getInstance().getPosition() > CalConstants.kElevatorLowSensitivityThreshold) {
									throttle *= CalConstants.kLowSensitivityFactor;
									turn *= CalConstants.kLowSensitivityFactor;
								}

								if (mDrive.getDriveControlState() == Drive.DriveControlState.OPEN_LOOP) {
									mDrive.setBrakeMode(driveJoystick.getRawButton(5) || driveJoystick.getRawButton(6) || VisionTracker.getInstance().isVisionEnabled());

									boolean quickTurn = driveJoystick.getRawButton(6) || VisionTracker.getInstance().isVisionEnabled();
									if (quickTurn)
										turn *= 0.5;

									mDrive.setOpenLoop(mCheesyDriveHelper.cheesyDrive(throttle, turn, quickTurn, true));
								}
							}

							if (driveJoystick.getRisingEdgeButton(1)) {
								TeleopActionRunner.runAction(AutomatedActions.enableHatchVision((t) -> driveJoystick.getRawButton(1)));
							}
							else if (driveJoystick.getRisingEdgeButton(2)) {
								TeleopActionRunner.runAction(AutomatedActions.enableHatchSidewaysVision((t) -> driveJoystick.getRawButton(2)));
							}
							else if (driveJoystick.getRisingEdgeTrigger(2, Constants.kJoystickTriggerThreshold)) {

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
//								TeleopActionRunner.runAction(AutomatedActions.rollerHatchFloorIntake((t) -> buttonBox1.getRawButton(12)));
							}
							else if (buttonBox1.getRisingEdgeButton(13)) {
								TeleopActionRunner.runAction(AutomatedActions.reverseHatchPickup());
							}
							else if (buttonBox1.getRisingEdgeButton(14)) {
								TeleopActionRunner.runAction(AutomatedActions.reverseHatchPlaceLow());
							}
							else if (buttonBox1.getRisingEdgeButton(15)) {
								TeleopActionRunner.runAction(AutomatedActions.reverseHatchPlaceLow());
							}
							else if (buttonBox1.getRisingEdgeButton(16)) {
								TeleopActionRunner.runAction(AutomatedActions.reverseHatchPlaceMid());
							}



							if (buttonBox2.getRisingEdgeButton(1)) {
								TeleopActionRunner.runAction(AutomatedActions.reverseHatchPlaceHigh());
							}
							else if (buttonBox2.getRisingEdgeButton(3)) {
								TeleopActionRunner.runAction(AutomatedActions.ballArmSet(BallIntakeArmPositions.Up));
							}
							else if (buttonBox2.getRisingEdgeButton(4)) {
								TeleopActionRunner.runAction(AutomatedActions.ballArmSet(BallIntakeArmPositions.Down));
							}
							else if (buttonBox2.getRisingEdgeButton(6)) {
								ConsoleReporter.report("Commanding Climb Lvl2 Action", MessageLevel.INFO);
								TeleopActionRunner.runAction(AutomatedActions.climbAutomatedLvl2((t) -> buttonBox2.getRawButton(6)));
							}
							else if (buttonBox2.getRisingEdgeButton(7)) {
								ConsoleReporter.report("Commanding Prepare Climb Action", MessageLevel.INFO);
//								BallIntakeArm.getInstance().configureClimbCurrentLimit();
//								Drive.getInstance().configureClimbCurrentLimit();
//								TeleopActionRunner.runAction(AutomatedActions.prepareClimb());
							}
							else if (buttonBox2.getRisingEdgeButton(8)) {

							}
							else if (buttonBox2.getRisingEdgeButton(9)) {
							    TeleopActionRunner.runAction(AutomatedActions.ballOuttake((t) -> buttonBox2.getRawButton(9)));
							}
							else if (buttonBox2.getRisingEdgeButton(10)) {
								TeleopActionRunner.runAction(AutomatedAction.fromAction(new SetBeakAction(false), 1));
							}
							else if (buttonBox2.getRisingEdgeButton(11)) {
								ConsoleReporter.report("Commanding Drive Mode", MessageLevel.INFO);
								TeleopActionRunner.runAction(AutomatedAction.fromAction(new SetDrivePTOAction(false), 1));
								mDrive.setOpenLoop(DriveSignal.NEUTRAL);
							}
							else if (buttonBox2.getRisingEdgeButton(12)) {
								ConsoleReporter.report("Commanding Climb Open Action", MessageLevel.INFO);
								TeleopActionRunner.runAction(AutomatedActions.climbOpen((t) -> buttonBox2.getRawButton(12), (t) -> -driveJoystick.getNormalizedAxis(1, 0.1), (t) -> -driveJoystick.getNormalizedAxis(5, 0.1)));
							}
							else if (buttonBox2.getRisingEdgeButton(13)) {
								ConsoleReporter.report("Commanding Automated Climb Action", MessageLevel.INFO);
								TeleopActionRunner.runAction(AutomatedActions.climbAutomated((t) -> buttonBox2.getRawButton(13)));
							}
							else if (buttonBox2.getRisingEdgeButton(14)) {
								//Flash LEDs
								LEDController.getInstance().setRequestedState(LEDController.LEDState.BLINK);
							}



							if (armControlJoystick.getRisingEdgeButton(1)) {
								//Flash LEDs to signal Human Player
								LEDController.getInstance().setLEDColor(Constants.kRequestGamePieceColor);
								LEDController.getInstance().setRequestedState(LEDController.LEDState.BLINK);
							}
							else if (armControlJoystick.getRisingEdgeButton(2)) {
								TeleopActionRunner.runAction(AutomatedAction.fromAction(new SetTurretPositionJoystickAction((t) -> armControlJoystick.getRawButton(2),
										(t) -> armControlJoystick.getNormalizedAxis(2, 0.1)), 300, Turret.getInstance()));
							}
							else if (armControlJoystick.getRisingEdgeButton(3)) {
								//Ball Outtake turret and arm
								TeleopActionRunner.runAction(AutomatedActions.shootBall((t) -> armControlJoystick.getRawButton(3)));
							}
							else if (armControlJoystick.getRisingEdgeButton(4)) {
								TeleopActionRunner.runAction(AutomatedActions.placeHatch());
							}
							else if (armControlJoystick.getRisingEdgeButton(5)) {
//								TeleopActionRunner.runAction(new AutomatedAction(new SetHatchPushAction(false), 1));
							}
							else if (armControlJoystick.getRisingEdgeButton(6)) {
								TeleopActionRunner.runAction(AutomatedActions.enableHatchVision((t) -> armControlJoystick.getRawButton(6)));
							}
							else if (armControlJoystick.getRisingEdgeButton(7)) {
								//Rehome Elevator
								TeleopActionRunner.runAction(AutomatedActions.homeElevator());
							}
							else if (armControlJoystick.getRisingEdgeButton(8)) {
								//Turret Open Loop
								TeleopActionRunner.runAction(AutomatedAction.fromAction(new SetTurretOpenLoopAction((t) -> armControlJoystick.getRawButton(8),
										(t) -> -armControlJoystick.getNormalizedAxis(2, 0.1) / 3.0), 300, Turret.getInstance()));
							}
							else if (armControlJoystick.getRisingEdgeButton(9)) {
								//Rehome Arm
							}
							else if (armControlJoystick.getRisingEdgeButton(11)) {
								//Rehome turret
								 Turret.getInstance().zeroSensors();
								 Turret.getInstance().setTurretControlMode(Turret.TurretControlMode.POSITION);
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
