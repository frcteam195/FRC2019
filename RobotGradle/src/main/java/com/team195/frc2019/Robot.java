package com.team195.frc2019;

import com.team195.frc2019.auto.AutoModeExecutor;
import com.team195.frc2019.loops.Looper;
import com.team195.frc2019.paths.TrajectoryGenerator;
import com.team195.frc2019.subsystems.*;
import com.team195.frc2019.auto.AutoModeBase;
import com.team195.lib.util.QuickMaths;
import com.team195.lib.drivers.dashjoy.CKDashJoystick;
import com.team195.lib.drivers.dashjoy.DashJoyReceiver;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.util.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Arrays;
import java.util.Optional;

public class Robot extends TimedRobot {
	private Looper mEnabledLooper = new Looper();
	private Looper mDisabledLooper = new Looper();
	private CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();
	private IControlBoard mControlBoard = ControlBoard.getInstance();
	private AutoFieldState mAutoFieldState = AutoFieldState.getInstance();
	private TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();

	private AutoModeSelector mAutoModeSelector = new AutoModeSelector();

	private CKDashJoystick dashjoy1 = new CKDashJoystick(0);

	private final SubsystemManager mSubsystemManager = new SubsystemManager(
			Arrays.asList(
					RobotStateEstimator.getInstance(),
					Drive.getInstance(),
					Infrastructure.getInstance()
			)
	);

	private Drive mDrive = Drive.getInstance();
	private LEDController mLED = LEDController.getInstance();
	private Infrastructure mInfrastructure = Infrastructure.getInstance();

	CKDashJoystick driveJoystick = new CKDashJoystick(0);

	private AutoModeExecutor mAutoModeExecutor;

	public Robot() {
		CrashTracker.logRobotConstruction();
	}

	@Override
	public void robotInit() {
		try {
			CrashTracker.logRobotInit();

			DashJoyReceiver.getInstance();

			mSubsystemManager.registerEnabledLoops(mEnabledLooper);
			mSubsystemManager.registerDisabledLoops(mDisabledLooper);

			mTrajectoryGenerator.generateTrajectories();

			mLED = LEDController.getInstance();
			mLED.start();
			mLED.setRequestedState(LEDController.LEDState.BLINK);

			ConnectionMonitor.getInstance().start();

			mAutoModeSelector.updateModeCreator();

			// Set the auto field state at least once.
			mAutoFieldState.setSides(DriverStation.getInstance().getGameSpecificMessage());
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void disabledInit() {
		SmartDashboard.putString("Match Cycle", "DISABLED");

		try {
			CrashTracker.logDisabledInit();
			mEnabledLooper.stop();
			if (mAutoModeExecutor != null) {
				mAutoModeExecutor.stop();
			}

			mInfrastructure.setIsDuringAuto(true);
			Drive.getInstance().zeroSensors();
			RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

			// Reset all auto mode state.
			mAutoModeSelector.reset();
			mAutoModeSelector.updateModeCreator();
			mAutoModeExecutor = new AutoModeExecutor();

			mDisabledLooper.start();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void autonomousInit() {
		SmartDashboard.putString("Match Cycle", "AUTONOMOUS");

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
	public void teleopInit() {
		SmartDashboard.putString("Match Cycle", "TELEOP");

		try {
			CrashTracker.logTeleopInit();
			mDisabledLooper.stop();
			if (mAutoModeExecutor != null) {
				mAutoModeExecutor.stop();
			}

			mAutoFieldState.disableOverride();

			mInfrastructure.setIsDuringAuto(false);

			RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
			mEnabledLooper.start();
			mDrive.setVelocity(DriveSignal.NEUTRAL, DriveSignal.NEUTRAL);
			mDrive.setOpenLoop(new DriveSignal(0.05, 0.05));
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void testInit() {
		SmartDashboard.putString("Match Cycle", "TEST");

		try {
			System.out.println("Starting check systems.");

			mDisabledLooper.stop();
			mEnabledLooper.stop();

			//mDrive.checkSystem();
			//mIntake.checkSystem();
			//mWrist.checkSystem();
//            mElevator.checkSystem();

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void disabledPeriodic() {
		SmartDashboard.putString("Match Cycle", "DISABLED");

		try {
			outputToSmartDashboard();
//            mWrist.resetIfAtLimit();
//            mElevator.resetIfAtLimit();

			// Poll FMS auto mode info and update mode creator cache
			mAutoFieldState.setSides(DriverStation.getInstance().getGameSpecificMessage());
			mAutoModeSelector.updateModeCreator();

			if (mAutoFieldState.isValid()) {
				Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode(mAutoFieldState);
				if (autoMode.isPresent() && autoMode.get() != mAutoModeExecutor.getAutoMode()) {
					System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
					mAutoModeExecutor.setAutoMode(autoMode.get());
				}
				System.gc();
			}
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void autonomousPeriodic() {
		SmartDashboard.putString("Match Cycle", "AUTONOMOUS");

		outputToSmartDashboard();
		try {

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void teleopPeriodic() {
		SmartDashboard.putString("Match Cycle", "TELEOP");
		double timestamp = Timer.getFPGATimestamp();

//        double throttle = mControlBoard.getThrottle();
//        double turn = mControlBoard.getTurn();

		double throttle = driveJoystick.getNormalizedAxis(1, 0.04);
		double turn = driveJoystick.getNormalizedAxis(4, 0.04);

		try {
			// When elevator is up, tune sensitivity on turn a little.
//            if (mElevator.getInchesOffGround() > Constants.kElevatorLowSensitivityThreshold) {
//                turn *= Constants.kLowSensitivityFactor;
//            }
			mDrive.setOpenLoop(mCheesyDriveHelper.cheesyDrive(throttle, turn, Controllers.getInstance().getDriveJoystickThrottle().getRawButton(5),
					mDrive.isHighGear()));


			outputToSmartDashboard();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void testPeriodic() {
		SmartDashboard.putString("Match Cycle", "TEST");
	}

	public void outputToSmartDashboard() {
		RobotState.getInstance().outputToSmartDashboard();
		Drive.getInstance().outputTelemetry();
		Infrastructure.getInstance().outputTelemetry();
		mAutoFieldState.outputToSmartDashboard();
		mEnabledLooper.outputToSmartDashboard();
		mAutoModeSelector.outputToSmartDashboard();
//        mCheesyVision2.outputTelemetry();
		// SmartDashboard.updateValues();
	}
}
