package com.team195.frc2019;

import com.team195.frc2019.auto.AutoModeExecutor;
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
import com.team254.lib.util.*;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

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
		HatchIntakeArm.getInstance(),
		Turret.getInstance(),
		Infrastructure.getInstance(),
		VisionTracker.getInstance()
	);

	private Drive mDrive = Drive.getInstance();
	private LEDController mLED = LEDController.getInstance();
	private Infrastructure mInfrastructure = Infrastructure.getInstance();
	private AutoModeExecutor mAutoModeExecutor;

	private HIDController mHIDController = HIDController.getInstance((t) -> mAutoModeExecutor);

	public Robot() {
		CrashTracker.logRobotConstruction();
		Thread.currentThread().setPriority(Constants.kRobotThreadPriority);
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
//		ConsoleReporter.report("LeftDrivePos:" + Drive.getInstance().getLeftEncoderDistance());
//		ConsoleReporter.report("BallIntakePos:"+BallIntakeArm.getInstance().getPosition());
	}

	@Override
	public void autonomousInit() {
		try {
			CrashTracker.logAutoInit();
			mDisabledLooper.stop();

			RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

			Drive.getInstance().zeroSensors();
			mInfrastructure.setIsDuringAuto(true);

			mAutoModeExecutor.start();

			mEnabledLooper.start();
			mHIDController.start();
		} catch (Throwable t) {
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
			if (mAutoModeExecutor != null) {
				mAutoModeExecutor.stop();
			}

			mInfrastructure.setIsDuringAuto(false);

			RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
			mEnabledLooper.start();
			mDrive.setVelocity(DriveSignal.NEUTRAL, DriveSignal.NEUTRAL);
			mDrive.setOpenLoop(new DriveSignal(0, 0));
			// mDrive.setBrakeMode(true);
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

			if (mSubsystemManager.checkSystemsPassDiagnostics())
				ConsoleReporter.report("System passed test!", MessageLevel.DEFCON1);
			else
				ConsoleReporter.report("System failed test!", MessageLevel.DEFCON1);

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
			mHIDController.stop();
			mEnabledLooper.stop();
			if (mAutoModeExecutor != null) {
				mAutoModeExecutor.stop();
			}

//			mInfrastructure.setIsDuringAuto(true);
			Drive.getInstance().zeroSensors();
			RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

			mAutoModeExecutor = new AutoModeExecutor();

			mDisabledLooper.start();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void disabledPeriodic() {
		try {
			mAutoModeExecutor.setAutoMode(mAutoModeSelector.getAutoMode());
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

}
