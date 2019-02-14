package com.team195.frc2019;

import com.team195.frc2019.auto.AutoModeExecutor;
import com.team195.frc2019.controllers.LEDController;
import com.team195.frc2019.loops.Looper;
import com.team195.frc2019.monitors.ConnectionMonitor;
import com.team195.frc2019.monitors.CriticalSystemsMonitor;
import com.team195.frc2019.paths.TrajectoryGenerator;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.reporters.LogDataReporter;
import com.team195.frc2019.reporters.MessageLevel;
import com.team195.frc2019.subsystems.*;
import com.team195.lib.drivers.dashjoy.DashJoyReceiver;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.util.*;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends TimedRobot {
	private Looper mEnabledLooper = new Looper("EnabledLooper");
	private Looper mDisabledLooper = new Looper("DisabledLooper");
	private CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();

	private AutoModeSelector mAutoModeSelector = new AutoModeSelector();

	private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance(
		RobotStateEstimator.getInstance(),
		Drive.getInstance(),
		Elevator.getInstance(),
		BallIntakeArm.getInstance(),
		HatchIntakeArm.getInstance(),
		Infrastructure.getInstance()
	);

	private Drive mDrive = Drive.getInstance();
	private LEDController mLED = LEDController.getInstance();
	private Infrastructure mInfrastructure = Infrastructure.getInstance();

	private AutoModeExecutor mAutoModeExecutor;

	public Robot() {
		CrashTracker.logRobotConstruction();
	}

	@Override
	public void robotInit() {
		try {
			CrashTracker.logRobotInit();
			ConsoleReporter.getInstance().start();

			DashJoyReceiver.getInstance();

			mSubsystemManager.registerEnabledLoops(mEnabledLooper);
			mSubsystemManager.registerDisabledLoops(mDisabledLooper);

			TrajectoryGenerator.getInstance().generateTrajectories();

			mLED.start();
			mLED.setRequestedState(LEDController.LEDState.BLINK);

			CriticalSystemsMonitor.getInstance();
			ConnectionMonitor.getInstance();
			LogDataReporter.getInstance();

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void robotPeriodic() {
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
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void teleopPeriodic() {
		double throttle = Controllers.getInstance().getDriveJoystick().getNormalizedAxis(1, 0.04);
		double turn = Controllers.getInstance().getDriveJoystick().getNormalizedAxis(4, 0.04);
		boolean quickTurn = Controllers.getInstance().getDriveJoystick().getRawButton(5);

		try {
			// When elevator is up, tune sensitivity on turn a little.
//            if (mElevator.getInchesOffGround() > Constants.kElevatorLowSensitivityThreshold) {
//                turn *= Constants.kLowSensitivityFactor;
//            }
			mDrive.setOpenLoop(mCheesyDriveHelper.cheesyDrive(throttle, turn, quickTurn, mDrive.isHighGear()));
			System.out.println("AvgVel:"+(Drive.getInstance().getLeftEncoderVelocityRPM() + Drive.getInstance().getLeftEncoderVelocityRPM())/2.0);


		} catch (Throwable t) {
			System.out.println(t.toString());
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void testInit() {
		try {
			ConsoleReporter.report("Starting check systems.", MessageLevel.INFO);

			mDisabledLooper.stop();
			mEnabledLooper.stop();

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
			mEnabledLooper.stop();
			if (mAutoModeExecutor != null) {
				mAutoModeExecutor.stop();
			}

			mInfrastructure.setIsDuringAuto(true);
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
//			mAutoModeExecutor.setAutoMode(mAutoModeSelector.getAutoMode());
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

}
