package com.team195.frc2019;

import com.team195.frc2019.auto.AutoModeExecutor;
import com.team195.frc2019.auto.autonomy.AutomatedActions;
import com.team195.frc2019.constants.Constants;
import com.team195.frc2019.constants.TestConstants;
import com.team195.frc2019.controllers.HIDController;
import com.team195.frc2019.controllers.LEDController;
import com.team195.frc2019.loops.Looper;
import com.team195.frc2019.monitors.ConnectionMonitor;
import com.team195.frc2019.monitors.CriticalSystemsMonitor;
import com.team195.frc2019.paths.TrajectoryGenerator;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.reporters.LogDataReporter;
import com.team195.frc2019.reporters.MessageLevel;
import com.team195.frc2019.subsystems.*;
import com.team195.lib.util.TeleopActionRunner;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.*;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import org.springframework.core.env.Environment;

public class Robot extends TimedRobot {
	private Looper mEnabledLooper = new Looper("EnabledLooper");
	private Looper mDisabledLooper = new Looper("DisabledLooper");

	private AutoModeSelector mAutoModeSelector = new AutoModeSelector();

	//All consume 20% CPU
	private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance(
		RobotStateEstimator.getInstance(), //10ms average
		Drive.getInstance(),        //0
		Elevator.getInstance(),     //20-40ms
		BallIntakeArm.getInstance(),
		Turret.getInstance(),
		Infrastructure.getInstance(),
		VisionTracker.getInstance()
	);

	private Drive mDrive = Drive.getInstance();
	private LEDController mLED = LEDController.getInstance();
	private Infrastructure mInfrastructure = Infrastructure.getInstance();
	public static AutoModeExecutor mAutoModeExecutor;

	private HIDController mHIDController = HIDController.getInstance();

	public Robot() {
		CrashTracker.logRobotConstruction();
		Thread.currentThread().setPriority(Constants.kRobotThreadPriority);
		Thread.currentThread().setName("RobotMainThread");
	}

	@Override
	public void robotInit() {
		try {
			CrashTracker.logRobotInit();

			LiveWindow.disableAllTelemetry();

			ConsoleReporter.getInstance().start();
			ConsoleReporter.setReportingLevel(MessageLevel.INFO);

			//Some weirdness with button detection, so disable for now
			//TODO: Fix dashboard DirectInput detection for button box
//			DashJoyReceiver.getInstance();

			mSubsystemManager.addAdditionalReportable(RobotState.getInstance());
			mSubsystemManager.registerEnabledLoops(mEnabledLooper);
			mSubsystemManager.registerDisabledLoops(mDisabledLooper);

			TrajectoryGenerator.getInstance().generateTrajectories();

			mLED.start();
			mLED.setRequestedState(LEDController.LEDState.BLINK);

			CriticalSystemsMonitor.getInstance();
			ConnectionMonitor.getInstance();
			TeleopActionRunner.init();

			Drive.getInstance().zeroSensors();
			RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

			LogDataReporter.getInstance();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void robotPeriodic() {
//		ConsoleReporter.report("ElevatorPos: " + Elevator.getInstance().getPosition());
//		ConsoleReporter.report(mEnabledLooper.generateReport());
//		ConsoleReporter.report("LeftDrivePos:" + Drive.getInstance().getLeftEncoderDistance() + ", RigthDrivePos:" + Drive.getInstance().getRightEncoderDistance());
//		ConsoleReporter.report("GyroRoll:" + Drive.getInstance().getRoll());
//		ConsoleReporter.report("Spark:" + Drive.getInstance().getRawLeftSparkEncoder() + ", Wheel:" + Drive.getInstance().getRawLeftEncoder());
//		ConsoleReporter.report("BallIntakePos:"+BallIntakeArm.getInstance().getPosition());
//		ConsoleReporter.report(mAutoModeSelector.getAutoMode().getClass().getSimpleName().toString());
	}

	@Override
	public void autonomousInit() {
		try {
			CrashTracker.logAutoInit();

			ConsoleReporter.report("Stopping Disabled Loops");
			mDisabledLooper.stop();

			ConsoleReporter.report("Set Infrastructure Auto");
			mInfrastructure.setIsDuringAuto(true);

			ConsoleReporter.report("Zero Drive Sensors");
			Drive.getInstance().zeroSensors();

			ConsoleReporter.report("Zero Pose");
			RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

//			Thread.sleep(500);

			ConsoleReporter.report("Check Auto Mode Not Null");
			if (mAutoModeExecutor != null) {
				ConsoleReporter.report("Start Auto Mode");
				mAutoModeExecutor.start();
			}

			ConsoleReporter.report("Start Enabled Looper");
			mEnabledLooper.start();

			ConsoleReporter.report("Start HIDController");
			mHIDController.start();
		} catch (Exception ex) {
			CrashTracker.logThrowableCrash(ex);
		}
		catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void autonomousPeriodic() {
		try {

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void teleopInit() {
		try {
			CrashTracker.logTeleopInit();
			mDisabledLooper.stop();

			if (mAutoModeExecutor != null)
				mAutoModeExecutor.stop();

			mInfrastructure.setIsDuringAuto(false);

			mEnabledLooper.start();
			mDrive.setVelocity(DriveSignal.NEUTRAL, DriveSignal.NEUTRAL);
			mDrive.setOpenLoop(new DriveSignal(0, 0));
			mHIDController.start();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void teleopPeriodic() {
		try {

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
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
				TeleopActionRunner.runAction(AutomatedActions.fullyAutomatedTest(), true);
			}

			//Crash the JVM and force a reset
			System.exit(1);
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
			CrashTracker.logDisabledInit();
			ConsoleReporter.report("Stopping HID");
			mHIDController.stop();
			ConsoleReporter.report("Stopping Enabled Looper");
			mEnabledLooper.stop();

			ConsoleReporter.report("Setting Brake Mode");
			mDrive.setBrakeMode(false);

			ConsoleReporter.report("Check Auto Mode Not Null");
			if (mAutoModeExecutor != null) {
				ConsoleReporter.report("Stop Auto Mode");
				mAutoModeExecutor.stop();
			}

			ConsoleReporter.report("Starting Disabled Looper");
			mDisabledLooper.start();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void disabledPeriodic() {
		try {
			mAutoModeExecutor = new AutoModeExecutor();
			mAutoModeExecutor.setAutoMode(mAutoModeSelector.getAutoMode());
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

}
