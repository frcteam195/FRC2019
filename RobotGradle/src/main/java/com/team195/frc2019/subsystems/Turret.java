package com.team195.frc2019.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.team195.frc2019.constants.CalConstants;
import com.team195.frc2019.RobotState;
import com.team195.frc2019.constants.DeviceIDConstants;
import com.team195.frc2019.loops.ILooper;
import com.team195.frc2019.loops.Loop;
import com.team195.frc2019.paths.TrajectoryGenerator;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.reporters.DiagnosticMessage;
import com.team195.frc2019.reporters.MessageLevel;
import com.team195.frc2019.reporters.ReflectingLogDataGenerator;
import com.team195.frc2019.subsystems.positions.BallIntakeArmPositions;
import com.team195.frc2019.subsystems.positions.ElevatorPositions;
import com.team195.frc2019.subsystems.positions.TurretPositions;
import com.team195.lib.drivers.CKSolenoid;
import com.team195.lib.drivers.motorcontrol.CKTalonSRX;
import com.team195.lib.drivers.motorcontrol.MCControlMode;
import com.team195.lib.drivers.motorcontrol.PDPBreaker;
import com.team195.lib.util.CachedValue;
import com.team195.lib.util.InterferenceSystem;
import com.team195.lib.util.MotionInterferenceChecker;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

public class Turret extends Subsystem implements InterferenceSystem {

	private static Turret mInstance = new Turret();

	private final VisionTracker mVisionTracker = VisionTracker.getInstance();

	private final CKTalonSRX mTurretRotationMotor;
	private final CKTalonSRX mBallShooterRollerMotor;
	private final CKSolenoid mHatchBeakSolenoid;
	private final CKSolenoid mHatchBeakFeedSolenoid;
	private final CKSolenoid mHatchPushSolenoid;
	private final CKSolenoid mBallPushSolenoid;

	private TurretControlMode mTurretControlMode = TurretControlMode.POSITION;
	private BallShooterControlMode mBallShooterControlMode = BallShooterControlMode.OPEN_LOOP;

	private AtomicBoolean beakListenerEnabled = new AtomicBoolean(true);

	private final MotionInterferenceChecker turretAnyPositionCheck;

	private PeriodicIO mPeriodicIO;
	private ReflectingLogDataGenerator<PeriodicIO> mLogDataGenerator = new ReflectingLogDataGenerator<>(PeriodicIO.class);


	private final CachedValue<Boolean> mTurretEncoderPresent;
	private final CachedValue<Boolean> mTurretMasterHasReset;

	private Turret() {
		mPeriodicIO = new PeriodicIO();

		//Encoder on 50:1
		//Turret gear is another 36:252
		mTurretRotationMotor = new CKTalonSRX(DeviceIDConstants.kTurretMotorId, false, PDPBreaker.B30A);
		mTurretRotationMotor.setInverted(true);
		mTurretRotationMotor.setSensorPhase(true);
		mTurretRotationMotor.setPIDF(CalConstants.kTurretPositionKp, CalConstants.kTurretPositionKi, CalConstants.kTurretPositionKd, CalConstants.kTurretPositionKf);
		mTurretRotationMotor.setMotionParameters(CalConstants.kTurretPositionCruiseVel, CalConstants.kTurretPositionMMAccel, CalConstants.kTurretPositionSCurveStrength);
		zeroSensors();
		mTurretRotationMotor.configForwardSoftLimitThreshold(CalConstants.kTurretForwardSoftLimit);
		mTurretRotationMotor.configForwardSoftLimitEnable(true);
		mTurretRotationMotor.configReverseSoftLimitThreshold(CalConstants.kTurretReverseSoftLimit);
		mTurretRotationMotor.configReverseSoftLimitEnable(true);
		mTurretRotationMotor.configCurrentLimit(CalConstants.kTurretContinuousCurrentLimit, CalConstants.kTurretPeakCurrentThreshold, CalConstants.kTurretPeakCurrentThresholdExceedDuration);
		mTurretRotationMotor.setControlMode(MCControlMode.MotionMagic);

//		TuneablePIDOSC x;
//		try {
//			x = new TuneablePIDOSC("Turret", 5804, true, mTurretRotationMotor);
//		} catch (Exception ignored) {
//
//		}

		mBallShooterRollerMotor = new CKTalonSRX(DeviceIDConstants.kBallShooterMotorId, false, PDPBreaker.B30A);
		mBallShooterRollerMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
		mBallShooterRollerMotor.setMCOpenLoopRampRate(CalConstants.kTurretBallShooterOpenLoopRamp);
		mBallShooterRollerMotor.configCurrentLimit(CalConstants.kTurretBallShooterContinuousCurrentLimit, CalConstants.kTurretBallShooterPeakCurrentThreshold, CalConstants.kTurretBallShooterPeakCurrentThresholdExceedDuration);

		mHatchBeakSolenoid = new CKSolenoid(DeviceIDConstants.kHatchBeakSolenoidId);
		mHatchBeakSolenoid.set(false);

		mHatchBeakFeedSolenoid = new CKSolenoid(DeviceIDConstants.kHatchBeakFeedSolenoidId);
		mHatchBeakFeedSolenoid.set(false);

		mHatchPushSolenoid = new CKSolenoid(DeviceIDConstants.kHatchPushSolenoidId);
		mHatchPushSolenoid.set(false);

		mBallPushSolenoid = new CKSolenoid(DeviceIDConstants.kBallPushSolenoidId);
		mBallPushSolenoid.setInverted(false);
		mBallPushSolenoid.set(false);

		turretAnyPositionCheck = new MotionInterferenceChecker(MotionInterferenceChecker.LogicOperation.AND, true,
				(t) -> (Elevator.getInstance().getPosition() >= ElevatorPositions.CollisionThresholdTurret - ElevatorPositions.PositionDelta),
				(t) -> (Elevator.getInstance().getSetpoint() >= ElevatorPositions.CollisionThresholdTurret),
				(t) -> (BallIntakeArm.getInstance().getSetpoint() == BallIntakeArmPositions.Down),
				(t) -> (BallIntakeArm.getInstance().getPosition() < BallIntakeArmPositions.CollisionThreshold)
		);

		mTurretEncoderPresent = new CachedValue<>(200, (t) -> mTurretRotationMotor.isEncoderPresent());
		mTurretMasterHasReset = new CachedValue<>(200, (t) -> mTurretRotationMotor.hasMotorControllerReset() != DiagnosticMessage.NO_MSG);
	}

	public static Turret getInstance() {
		return mInstance;
	}

	@Override
	public void stop() {
		mTurretRotationMotor.set(MCControlMode.PercentOut, 0, 0, 0);
	}

	@Override
	public synchronized boolean isSystemFaulted() {
		boolean systemFaulted = !mPeriodicIO.turret_encoder_present;

		if (systemFaulted) {
			ConsoleReporter.report("Turret Encoder Missing!", MessageLevel.DEFCON1);
		}

		systemFaulted |= mPeriodicIO.turret_reset;

		if (systemFaulted) {
			ConsoleReporter.report("Turret Requires Rehoming!", MessageLevel.DEFCON1);
			setTurretControlMode(TurretControlMode.DISABLED);
		}

		return systemFaulted;
	}

	@Override
	public boolean runDiagnostics() {
		return false;
	}

	@Override
	public synchronized List<Object> generateReport() {
		return mLogDataGenerator.generateData(mPeriodicIO);
	}

	@Override
	public void zeroSensors() {
		mTurretRotationMotor.setEncoderPosition(0);
		setTurretPosition(0);
		if (mTurretControlMode == TurretControlMode.POSITION)
			mTurretRotationMotor.set(MCControlMode.MotionMagic, 0, 0, 0);
	}

	@Override
	public void registerEnabledLoops(ILooper in) {
		in.register(mLoop);
	}

	private final Loop mLoop = new Loop() {
		@Override
		public void onFirstStart(double timestamp) {
			synchronized (Turret.this) {
//				zeroSensors();
			}
		}

		@Override
		public void onStart(double timestamp) {
			synchronized (Turret.this) {

			}
		}

		@SuppressWarnings("Duplicates")
		@Override
		public void onLoop(double timestamp) {
			synchronized (Turret.this) {
				switch (mTurretControlMode) {
					case AUTO_TRACK:
						Pose2d robotCurrentPos = RobotState.getInstance().getLatestFieldToVehicle().getValue();
						Translation2d currentRocketTarget;

						if (robotCurrentPos.getTranslation().x() > 0)
							//Track Left Rocket
							currentRocketTarget = TrajectoryGenerator.kLeftRocketPose.getTranslation();
						else
							//Track Right Rocket
							currentRocketTarget = TrajectoryGenerator.kRightRocketPose.getTranslation();

						double desiredTurretAngleDeg = Math.toDegrees(Math.atan2((currentRocketTarget.y() - robotCurrentPos.getTranslation().y()),
								(currentRocketTarget.x() - robotCurrentPos.getTranslation().x()))) - robotCurrentPos.getRotation().getDegrees();

						mPeriodicIO.turret_setpoint = convertTurretDegreesToRotations(desiredTurretAngleDeg);
						//Fall through on purpose to set position -> no break;
					case VISION_TRACK:
						//Preempts Auto Track
						if (mVisionTracker.isTargetFound())
							mPeriodicIO.turret_setpoint = convertTurretDegreesToRotations(mVisionTracker.getTargetHorizAngleDev());
						//Fall through on purpose to set position -> no break;
					case POSITION:
						if (turretAnyPositionCheck.hasPassedConditions() || Elevator.getInstance().getPosition() > ElevatorPositions.CargoBall
							|| Math.abs(mPeriodicIO.turret_setpoint - TurretPositions.Back180) < Turret.convertTurretDegreesToRotations(10)
							|| Math.abs(mPeriodicIO.turret_setpoint - TurretPositions.Home) < Turret.convertTurretDegreesToRotations(10))
							mTurretRotationMotor.set(MCControlMode.MotionMagic, mPeriodicIO.turret_setpoint, 0, 0);
//						else if (mPeriodicIO.turret_setpoint != TurretPositions.Back180 && mPeriodicIO.turret_setpoint != TurretPositions.Home)
//							mTurretRotationMotor.set(MCControlMode.MotionMagic, 0, 0, 0);
						break;
					case OPEN_LOOP:
						mTurretRotationMotor.set(MCControlMode.PercentOut, Math.min(Math.max(mPeriodicIO.turret_setpoint, -1), 1), 0, 0);
						break;
					default:
						mTurretRotationMotor.set(MCControlMode.Disabled, 0, 0, 0);
						break;
				}

				switch (mBallShooterControlMode) {
					case VELOCITY:
						mBallShooterRollerMotor.set(MCControlMode.SmartVelocity, mPeriodicIO.ball_shooter_setpoint, 0, 0);
						break;
					case CURRENT:
						mBallShooterRollerMotor.set(MCControlMode.Current, mPeriodicIO.ball_shooter_setpoint, 0, 0);
						break;
					case OPEN_LOOP:
						mBallShooterRollerMotor.set(MCControlMode.PercentOut, Math.min(Math.max(mPeriodicIO.ball_shooter_setpoint, -1), 1), 0, 0);
						break;
					default:
						break;
				}
			}
		}

		@Override
		public void onStop(double timestamp) {
			stop();
		}

		@Override
		public String getName() {
			return "Turret";
		}
	};

	public synchronized boolean getLimitSwitchValue() {
		return mPeriodicIO.hatch_limit_switch;
	}

	public synchronized boolean getLimitSwitchFallingEdge() { return mBallShooterRollerMotor.getReverseLimitFallingEdge(); }

	public boolean isBeakListenerEnabled() { return beakListenerEnabled.get(); }

	public void setBeakListened(boolean enabled) {
		beakListenerEnabled.set(enabled);
	}

	public synchronized void setBallPush(boolean ballPush) {
		mBallPushSolenoid.set(ballPush);
	}

	public synchronized void setHatchPush(boolean hatchPush) {
		mHatchPushSolenoid.set(hatchPush);
	}

	public synchronized void setBeak(boolean closed) {
		mHatchBeakSolenoid.set(closed);
	}

	public synchronized void setBeakFeedOff(boolean off) {
		mHatchBeakFeedSolenoid.set(off);
	}

	public synchronized void setTurretPosition(double turretPosition) {
		mPeriodicIO.turret_setpoint = turretPosition;
	}

	public synchronized void setBallShooterCurrent(double ballShooterCurrent) {
		setBallShooterControlMode(BallShooterControlMode.CURRENT);
		mPeriodicIO.ball_shooter_setpoint = ballShooterCurrent;
	}

	public synchronized void setBallShooterVelocity(double ballShooterVelocity) {
		setBallShooterControlMode(BallShooterControlMode.VELOCITY);
		mPeriodicIO.ball_shooter_setpoint = ballShooterVelocity;
	}

	public synchronized void setBallShooterOpenLoop(double ballShooterOutput) {
		setBallShooterControlMode(BallShooterControlMode.OPEN_LOOP);
		mPeriodicIO.ball_shooter_setpoint = ballShooterOutput;
	}

	public synchronized void setTurretControlMode(TurretControlMode turretControlMode) {
		if (mTurretControlMode != turretControlMode)
			mTurretControlMode = turretControlMode;
	}

	private synchronized void setBallShooterControlMode(BallShooterControlMode ballShooterControlMode) {
		if (mBallShooterControlMode != ballShooterControlMode)
			mBallShooterControlMode = ballShooterControlMode;
	}

	public boolean isTurretAtSetpoint(double posDelta) {
		return Math.abs(mPeriodicIO.turret_setpoint - mPeriodicIO.turret_position) < Math.abs(posDelta);
	}

	public static double convertRotationsToTurretDegrees(double rotations) {
		return rotations / (CalConstants.kTurretLargeGearTeeth / CalConstants.kTurretSmallGearTeeth / 360.0);
	}

	public static double convertTurretDegreesToRotations(double degrees) {
		return degrees * (CalConstants.kTurretLargeGearTeeth / CalConstants.kTurretSmallGearTeeth / 360.0);
	}

	@Override
	public double getPosition() {
		return mPeriodicIO.turret_position;
	}

	@Override
	public double getSetpoint() {
		return mPeriodicIO.turret_setpoint;
	}

	public enum TurretControlMode {
		POSITION,
		OPEN_LOOP,
		AUTO_TRACK,
		VISION_TRACK,
		DISABLED;
	}

	public enum BallShooterControlMode {
		VELOCITY,
		CURRENT,
		OPEN_LOOP;
	}

	@Override
	public synchronized void readPeriodicInputs() {
		mPeriodicIO.turret_position = mTurretRotationMotor.getPosition();
		mPeriodicIO.turret_encoder_present = mTurretEncoderPresent.getValue();
		mPeriodicIO.turret_reset = mTurretMasterHasReset.getValue();
		mPeriodicIO.hatch_limit_switch = mBallShooterRollerMotor.getReverseLimitValue();
	}

	@Override
	public synchronized void writePeriodicOutputs() {

	}

	@SuppressWarnings("WeakerAccess")
	public static class PeriodicIO {
		//Making members public here will automatically add them to logs
		// INPUTS
		public double turret_position;
		public double turret_setpoint;
		public double ball_shooter_setpoint;
		public boolean turret_encoder_present;
		public boolean turret_reset;
		public boolean hatch_limit_switch;
	}
}