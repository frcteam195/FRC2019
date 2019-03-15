package com.team195.frc2019.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.team195.frc2019.Constants;
import com.team195.frc2019.loops.ILooper;
import com.team195.frc2019.loops.Loop;
import com.team195.frc2019.reporters.DiagnosticMessage;
import com.team195.frc2019.subsystems.positions.ElevatorPositions;
import com.team195.frc2019.subsystems.positions.TurretPositions;
import com.team195.lib.drivers.CKSolenoid;
import com.team195.lib.drivers.motorcontrol.CKTalonSRX;
import com.team195.lib.drivers.motorcontrol.MCControlMode;
import com.team195.lib.drivers.motorcontrol.PDPBreaker;
import com.team195.lib.util.InterferenceSystem;
import com.team195.lib.util.MotionInterferenceChecker;
import com.team195.lib.util.ThreadRateControl;

public class BallIntakeArm extends Subsystem implements InterferenceSystem {

	private static BallIntakeArm mInstance = new BallIntakeArm();

	private final CKTalonSRX mBallArmRotationMotor;
	private final CKTalonSRX mBallArmRollerMotor;
	private final ThreadRateControl trc = new ThreadRateControl();

	private final MotionInterferenceChecker ballArmUpCheck;

	private final CKSolenoid mBallIntakeBarDropSolenoid;

	private BallIntakeArmControlMode mBallIntakeArmControlMode = BallIntakeArmControlMode.DISABLED;

	private double mBallIntakeArmSetpoint = 0;
	private double mBallIntakeRollerSetpoint = 0;

	private BallIntakeArm() {
		mBallArmRotationMotor = new CKTalonSRX(Constants.kBallIntakeRotationMotorId, false, PDPBreaker.B30A);
		mBallArmRotationMotor.setSensorPhase(true);

		mBallArmRollerMotor = new CKTalonSRX(Constants.kBallIntakeRollerMotorId, false, PDPBreaker.B30A);
		mBallArmRollerMotor.setInverted(true);
		mBallArmRollerMotor.setSensorPhase(true);
		mBallArmRollerMotor.setMCOpenLoopRampRate(0.2);

		mBallArmRotationMotor.setPIDGainSlot(0);
		mBallArmRotationMotor.setFeedbackDevice(FeedbackDevice.CTRE_MagEncoder_Relative);
		mBallArmRotationMotor.setPIDF(Constants.kBallIntakeArmUpPositionKp, Constants.kBallIntakeArmUpPositionKi, Constants.kBallIntakeArmUpPositionKd, Constants.kBallIntakeArmUpPositionKf);
		mBallArmRotationMotor.setMotionParameters(Constants.kBallIntakeArmUpPositionCruiseVel, Constants.kBallIntakeArmUpPositionMMAccel);
		mBallArmRotationMotor.setPIDGainSlot(1);
		mBallArmRotationMotor.setFeedbackDevice(RemoteFeedbackDevice.RemoteSensor0, Constants.kBallIntakeRollerMotorId);
		mBallArmRotationMotor.setPIDF(Constants.kBallIntakeArmDownPositionKp, Constants.kBallIntakeArmDownPositionKi, Constants.kBallIntakeArmDownPositionKd, Constants.kBallIntakeArmDownPositionKf);
		mBallArmRotationMotor.setMotionParameters(Constants.kBallIntakeArmDownPositionCruiseVel, Constants.kBallIntakeArmDownPositionMMAccel);

		mBallArmRotationMotor.setPIDGainSlot(0);
		trc.start();
		trc.doRateControl(100);
		zeroSensors();
		trc.doRateControl(100);
		mBallArmRotationMotor.configForwardSoftLimitThreshold(Constants.kBallIntakeArmForwardSoftLimit);
		mBallArmRotationMotor.configForwardSoftLimitEnable(true);
		mBallArmRotationMotor.configReverseSoftLimitEnable(false);
		mBallArmRotationMotor.configCurrentLimit(6, 0, 0);
		mBallArmRotationMotor.setControlMode(MCControlMode.Disabled);

//		TuneablePIDOSC x;
//		try {
//			x = new TuneablePIDOSC("Ball Arm", 5804, true, mBallArmRotationMotor);
//		} catch (Exception ignored) {
//
//		}

		ballArmUpCheck = new MotionInterferenceChecker(MotionInterferenceChecker.LogicOperation.AND, true,
				(t) -> Elevator.getInstance().getPosition() < ElevatorPositions.CollisionThresholdBallArm,
				(t) -> Math.abs(Turret.getInstance().getPosition()) < Math.abs(TurretPositions.Home - TurretPositions.PositionDelta)
		);

		mBallIntakeBarDropSolenoid = new CKSolenoid(Constants.kBallIntakeBarSolenoidId);
	}

	public static BallIntakeArm getInstance() {
		return mInstance;
	}

	@Override
	public void stop() {

	}

	@Override
	public boolean isSystemFaulted() {
		boolean systemFaulted = !mBallArmRotationMotor.isEncoderPresent();
		systemFaulted |= mBallArmRotationMotor.hasMotorControllerReset() != DiagnosticMessage.NO_MSG;
		if (systemFaulted)
			setBallIntakeArmControlMode(BallIntakeArmControlMode.DISABLED);
		return systemFaulted;
	}

	@Override
	public boolean runDiagnostics() {
		return false;
	}

	@Override
	public String generateReport() {
		return  "BallArmPos:" + mBallArmRotationMotor.getVelocity() + ";" +
				"BallArmVel:" + mBallArmRotationMotor.getVelocity() + ";" +
				"BallArmOutput:" + mBallIntakeArmSetpoint + ";" +
				"BallArmCurrent:" + mBallArmRotationMotor.getMCOutputCurrent() + ";" +
				"BallArmOutputDutyCycle:" + mBallArmRotationMotor.getMCOutputPercent() + ";" +
				"BallArmOutputVoltage:" + mBallArmRotationMotor.getMCOutputPercent() * mBallArmRotationMotor.getMCInputVoltage() + ";" +
				"BallArmSupplyVoltage:" + mBallArmRotationMotor.getMCInputVoltage() + ";" +
				"BallArmControlMode:" + mBallIntakeArmControlMode.toString() + ";" +
				"BallArmRotationMotorHasReset:" + mBallArmRotationMotor.hasMotorControllerReset().getMessage() + ";" +
				"BallArmRollerMotorHasReset:" + mBallArmRollerMotor.hasMotorControllerReset().getMessage() + ";" +
				"BallArmIntakeCurrent:" + mBallArmRollerMotor.getMCOutputCurrent() + ";" +
				"BallArmIntakeOutputDutyCycle:" + mBallArmRollerMotor.getMCOutputPercent() + ";" +
				"BallArmIntakeOutputVoltage:" + mBallArmRollerMotor.getMCOutputPercent() * mBallArmRollerMotor.getMCInputVoltage() + ";" +
				"BallArmIntakeSupplyVoltage:" + mBallArmRollerMotor.getMCInputVoltage() + ";";
	}

	@Override
	public void zeroSensors() {
		mBallArmRotationMotor.setEncoderPosition(Constants.kBallIntakeArmForwardSoftLimit);
		zeroRemoteSensor();
	}

	public void zeroRemoteSensor() {
		mBallArmRollerMotor.setEncoderPosition(0);
	}

	public void setSensorsForReset() {
		mBallArmRotationMotor.setLocalQuadPosition(0);
		mBallArmRotationMotor.setEncoderPosition(0);
		trc.doRateControl(100);
	}

	@Override
	public void registerEnabledLoops(ILooper in) {
		in.register(mLoop);
	}

	private final Loop mLoop = new Loop() {
		@Override
		public void onFirstStart(double timestamp) {
			synchronized (BallIntakeArm.this) {
				zeroSensors();

//				(new TeleopActionRunner(AutomatedActions.unfold())).runAction();
			}
		}

		@Override
		public void onStart(double timestamp) {
			synchronized (BallIntakeArm.this) {

			}
		}

		@SuppressWarnings("Duplicates")
		@Override
		public void onLoop(double timestamp) {
			synchronized (BallIntakeArm.this) {
				switch (mBallIntakeArmControlMode) {
					case POSITION:
						int rotationPIDSlot = 0;
						if (mBallIntakeArmSetpoint <= 0)
							rotationPIDSlot = 1;

						if (ballArmUpCheck.hasPassedConditions())
							mBallArmRotationMotor.set(MCControlMode.MotionMagic, mBallIntakeArmSetpoint, rotationPIDSlot, 0);

						break;
					case OPEN_LOOP:
						mBallArmRotationMotor.set(MCControlMode.PercentOut, Math.min(Math.max(mBallIntakeArmSetpoint, -1), 1), 0, 0);
						break;
					case DISABLED:
					default:
						mBallArmRotationMotor.setControlMode(MCControlMode.Disabled);
						break;
				}

				mBallArmRollerMotor.set(MCControlMode.PercentOut, Math.min(Math.max(mBallIntakeRollerSetpoint, -1), 1), 0, 0);
			}
		}

		@Override
		public void onStop(double timestamp) {
			stop();
		}

		@Override
		public String getName() {
			return "BallIntakeArm";
		}
	};

	public void configureClimbCurrentLimit() {
//		mBallArmRollerMotor.configCurrentLimit();
		mBallArmRotationMotor.configCurrentLimit(10, 25, 250);
		mBallArmRotationMotor.configForwardSoftLimitEnable(false);
		mBallArmRotationMotor.configReverseSoftLimitEnable(false);
	}

	public void dropClimbBar() {
		mBallIntakeBarDropSolenoid.set(true);
	}

	@Override
	public double getPosition() {
		return mBallArmRotationMotor.getLocalQuadPosition();
	}

	public double getRemotePosition() {
		return mBallArmRollerMotor.getPosition();
	}

	@Override
	public double getSetpoint() {
		return mBallIntakeArmSetpoint;
	}

	public synchronized void setBallIntakeArmPosition(double armPosition) {
		mBallIntakeArmSetpoint = armPosition;
	}

	public synchronized void setBallIntakeRollerSpeed(double rollerSpeed) {
		mBallIntakeRollerSetpoint = rollerSpeed;
	}

	public boolean isArmAtSetpoint(double posDelta) {
		return Math.abs(mBallIntakeArmSetpoint - mBallArmRotationMotor.getPosition()) < Math.abs(posDelta);
	}

	public synchronized void setBallIntakeArmControlMode(BallIntakeArmControlMode ballIntakeArmControlMode) {
		mBallIntakeArmControlMode = ballIntakeArmControlMode;
	}

	public enum BallIntakeArmControlMode {
		POSITION,
		OPEN_LOOP,
		DISABLED;
	}
}
