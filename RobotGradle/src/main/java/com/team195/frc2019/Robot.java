package com.team195.frc2019;

import com.team195.frc2019.auto.AutoModeExecutor;
import com.team195.frc2019.auto.autonomy.AutomatedActions;
import com.team195.frc2019.constants.Constants;
import com.team195.frc2019.constants.TestConstants;
import com.team195.frc2019.controllers.HIDController;
import com.team195.frc2019.controllers.LEDController;
import com.team195.frc2019.loops.Looper;
import com.team195.frc2019.monitors.ConnectionMonitor;
import com.team195.frc2019.paths.TrajectoryGenerator;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.reporters.MessageLevel;
import com.team195.frc2019.subsystems.*;
import com.team195.lib.util.TeleopActionRunner;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.util.CrashTracker;
import com.team254.lib.util.DriveSignal;
import edu.wpi.first.wpilibj.Timer;

@SuppressWarnings({"ResultOfMethodCallIgnored", "FieldCanBeLocal", "WeakerAccess"})
public class Robot extends RealtimeRobot {
	private Looper mEnabledLooper = new Looper("EnabledLooper");
	private Looper mDisabledLooper = new Looper("DisabledLooper");

	private AutoModeSelector mAutoModeSelector = new AutoModeSelector();

	private SubsystemManager mSubsystemManager;

	private Drive mDrive;
	private LEDController mLED;
	private Infrastructure mInfrastructure;
	public static final AutoModeExecutor mAutoModeExecutor = new AutoModeExecutor();

	private HIDController mHIDController;

	public Robot() {
		CrashTracker.logRobotConstruction();
		Thread.currentThread().setPriority(Constants.kRobotThreadPriority);
		Thread.currentThread().setName("RobotMainThread");
	}

	@Override
	public void robotInit() {
		try {
			CrashTracker.logRobotInit();

			mDrive = Drive.getInstance();
			mLED = LEDController.getInstance();
			mInfrastructure = Infrastructure.getInstance();
			mHIDController = HIDController.getInstance();

			mSubsystemManager = SubsystemManager.getInstance(
				RobotStateEstimator.getInstance(),
				Drive.getInstance(),
				Elevator.getInstance(),
				BallIntakeArm.getInstance(),
				Turret.getInstance(),
				Infrastructure.getInstance(),
				VisionTracker.getInstance()
			);

			ConsoleReporter.getInstance();
			ConsoleReporter.setReportingLevel(MessageLevel.INFO);

			mSubsystemManager.registerEnabledLoops(mEnabledLooper);
			mSubsystemManager.registerDisabledLoops(mDisabledLooper);

			TrajectoryGenerator.getInstance().generateTrajectories();

			mLED.start();
			mLED.setRequestedState(LEDController.LEDState.BLINK);

			ConnectionMonitor.getInstance();

			Drive.getInstance().zeroSensors();
			RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
			SensorFusedRobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

			System.gc();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void robotPeriodic() {
//		ConsoleReporter.report("ElevatorPos: " + Elevator.getInstance().getPosition());
//		ConsoleReporter.report(mEnabledLooper.generateReport());
//		ConsoleReporter.report(mDisabledLooper.generateReport());
//		ConsoleReporter.report("LeftDrivePos:" + Drive.getInstance().getLeftEncoderDistance() + ", RigthDrivePos:" + Drive.getInstance().getRightEncoderDistance());
//		ConsoleReporter.report("GyroRoll:" + Drive.getInstance().getRoll());
//		ConsoleReporter.report("Spark:" + Drive.getInstance().getRawLeftSparkEncoder() + ", Wheel:" + Drive.getInstance().getRawLeftEncoder());
//		ConsoleReporter.report("BallIntakePos:"+BallIntakeArm.getInstance().getPosition());
//		ConsoleReporter.report(mAutoModeSelector.getAutoMode().getClass().getSimpleName().toString());
//		ConsoleReporter.report("GyroDeg:" + Drive.getInstance().getRawYaw());
//		ConsoleReporter.report("Skew: " + VisionTracker.getInstance().getTargetSkew());
	}

	@Override
	public void autonomousInit() {
		teleopInit();
//		try {
//			CrashTracker.logAutoInit();
//
//			mDisabledLooper.stop();
//			mInfrastructure.setIsDuringAuto(true);
//			Drive.getInstance().zeroSensors();
//			RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
//			mDrive.setBrakeMode(true);
//			mDrive.forceBrakeModeUpdate();
//			mDrive.setVelocity(DriveSignal.NEUTRAL, DriveSignal.NEUTRAL);
//			mEnabledLooper.start();
//			mHIDController.start();
//
//			if (mAutoModeExecutor.isSet()) {
//				ConsoleReporter.report("Start Auto Mode");
//				mAutoModeExecutor.start();
//			}
//		} catch (Exception ex) {
//			CrashTracker.logThrowableCrash(ex);
//		}
//		catch (Throwable t) {
//			CrashTracker.logThrowableCrash(t);
//			throw t;
//		}
	}

	@Override
	public void autonomousPeriodic() {
//		try {
//
//		} catch (Throwable t) {
//			CrashTracker.logThrowableCrash(t);
//			throw t;
//		}
	}

	@Override
	public void teleopInit() {
		try {
//			CrashTracker.logTeleopInit();
			mDisabledLooper.stop();

			if (mAutoModeExecutor.isSet()) {
				try {
					mAutoModeExecutor.stop();
				} catch (Exception ignored) {

				}
			}

			mInfrastructure.setIsDuringAuto(false);

			mEnabledLooper.start();
			mDrive.setVelocity(DriveSignal.NEUTRAL, DriveSignal.NEUTRAL);
			mDrive.setOpenLoop(DriveSignal.NEUTRAL);
			mDrive.setBrakeMode(false);
			mDrive.forceBrakeModeUpdate();
			mHIDController.start();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void teleopPeriodic() {
//		try {
//
//		} catch (Throwable t) {
//			CrashTracker.logThrowableCrash(t);
//			throw t;
//		}
	}

	@Override
	public void testInit() {
		try {
			ConsoleReporter.report("Starting systems check!", MessageLevel.INFO);

			mDisabledLooper.stop();
			mEnabledLooper.stop();
			mHIDController.stop();

			if (TestConstants.RUN_INDIVIDUAL_TESTS) {
				if (mSubsystemManager.checkSystemsPassDiagnostics())
					ConsoleReporter.report("System passed test!", MessageLevel.DEFCON1);
				else
					ConsoleReporter.report("System failed test!", MessageLevel.DEFCON1);
			}
			else {
				mEnabledLooper.start();
				TeleopActionRunner.runAction(AutomatedActions.fullyAutomatedTest());
			}

			//Crash the JVM and force a reset
//			System.exit(1);
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void disabledInit() {
		try {
//			CrashTracker.logDisabledInit();

			mHIDController.stop();
			mEnabledLooper.stop();
			mDrive.setBrakeMode(false);
			mDrive.forceBrakeModeUpdate();
			if (mAutoModeExecutor.isSet()) {
				try {
					mAutoModeExecutor.stop();
				} catch (Exception ignored) {

				}
			}
			mDisabledLooper.start();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void disabledPeriodic() {
		try {
			mAutoModeExecutor.reset();
//			mAutoModeExecutor.setAutoMode(mAutoModeSelector.getAutoMode());
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

}
