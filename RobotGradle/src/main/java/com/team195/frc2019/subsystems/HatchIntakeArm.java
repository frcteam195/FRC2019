package com.team195.frc2019.subsystems;

import com.team195.frc2019.Constants;
import com.team195.frc2019.loops.ILooper;
import com.team195.frc2019.loops.Loop;
import com.team195.frc2019.reporters.DiagnosticMessage;
import com.team195.frc2019.subsystems.positions.ElevatorPositions;
import com.team195.frc2019.subsystems.positions.HatchArmPositions;
import com.team195.lib.drivers.motorcontrol.CKTalonSRX;
import com.team195.lib.drivers.motorcontrol.MCControlMode;
import com.team195.lib.drivers.motorcontrol.PDPBreaker;
import com.team195.lib.util.InterferenceSystem;
import com.team195.lib.util.MotionInterferenceChecker;

public class HatchIntakeArm extends Subsystem implements InterferenceSystem {

	private static HatchIntakeArm mInstance = new HatchIntakeArm();

	private final CKTalonSRX mHatchArmRotationMotor;
	private final CKTalonSRX mHatchArmRollerMotor;

	private final MotionInterferenceChecker hatchArmAnyPositionCheck;
	private final MotionInterferenceChecker hatchArmPauseDownCheck;
	private final MotionInterferenceChecker hatchArmPauseUpCheck;

	private HatchArmControlMode mHatchArmControlMode = HatchArmControlMode.POSITION;

	private double mHatchArmSetpoint = 0;
	private double mHatchRollerSetpoint = 0;

	private HatchIntakeArm() {
		mHatchArmRotationMotor = new CKTalonSRX(Constants.kHatchIntakeRotationMotorId, false, PDPBreaker.B30A);
		mHatchArmRotationMotor.setInverted(true);
		mHatchArmRotationMotor.setSensorPhase(true);
		mHatchArmRotationMotor.setPIDF(Constants.kHatchArmPositionKp, Constants.kHatchArmPositionKi, Constants.kHatchArmPositionKd, Constants.kHatchArmPositionKf);
		mHatchArmRotationMotor.setMotionParameters(Constants.kHatchArmPositionCruiseVel, Constants.kHatchArmPositionMMAccel);
		zeroSensors();
		mHatchArmRotationMotor.configForwardSoftLimitThreshold(Constants.kHatchArmForwardSoftLimit);
		mHatchArmRotationMotor.configForwardSoftLimitEnable(true);
		mHatchArmRotationMotor.configReverseSoftLimitThreshold(Constants.kHatchArmReverseSoftLimit);
		mHatchArmRotationMotor.configReverseSoftLimitEnable(true);
		mHatchArmRotationMotor.configCurrentLimit(10, 15, 250);
		mHatchArmRotationMotor.setControlMode(MCControlMode.MotionMagic);

//		TuneablePIDOSC x;
//		try {
//			x = new TuneablePIDOSC("Hatch Arm", 5804, true, mHatchArmRotationMotor);
//		} catch (Exception ignored) {
//
//		}

		mHatchArmRollerMotor = new CKTalonSRX(Constants.kHatchIntakeRollerMotorId, false, PDPBreaker.B30A);
		mHatchArmRollerMotor.setInverted(true);
		mHatchArmRollerMotor.setMCOpenLoopRampRate(0.2);
		mHatchArmRollerMotor.configCurrentLimit(10, 25, 250);


		hatchArmAnyPositionCheck = new MotionInterferenceChecker(MotionInterferenceChecker.LogicOperation.AND, true,
				(t) -> (Elevator.getInstance().getPosition() > ElevatorPositions.CollisionThresholdHatchArm)
		);

		hatchArmPauseDownCheck = new MotionInterferenceChecker(MotionInterferenceChecker.LogicOperation.AND, true,
				(t) -> (mHatchArmSetpoint < HatchArmPositions.CollisionThreshold),
				(t) -> (getPosition() > HatchArmPositions.CollisionThreshold + HatchArmPositions.PositionDelta)
		);

		hatchArmPauseUpCheck = new MotionInterferenceChecker(MotionInterferenceChecker.LogicOperation.AND, true,
				(t) -> (mHatchArmSetpoint >= HatchArmPositions.Inside),
				(t) -> (getPosition() < HatchArmPositions.PositionDelta)
		);
	}

	public static HatchIntakeArm getInstance() {
		return mInstance;
	}

	@Override
	public void stop() {
		mHatchArmRotationMotor.set(MCControlMode.PercentOut, 0, 0, 0);
	}

	@Override
	public boolean isSystemFaulted() {
		boolean systemFaulted = !mHatchArmRotationMotor.isEncoderPresent();
		systemFaulted |= mHatchArmRotationMotor.hasMotorControllerReset() != DiagnosticMessage.NO_MSG;
		if (systemFaulted)
			setHatchArmControlMode(HatchArmControlMode.DISABLED);
		return systemFaulted;
	}

	@Override
	public boolean runDiagnostics() {
		return false;
	}

	@Override
	public String generateReport() {
		return  "HatchArmPos:" + mHatchArmRotationMotor.getVelocity() + ";" +
				"HatchArmVel:" + mHatchArmRotationMotor.getVelocity() + ";" +
				"HatchArmOutput:" + mHatchArmSetpoint + ";" +
				"HatchArmCurrent:" + mHatchArmRotationMotor.getMCOutputCurrent() + ";" +
				"HatchArmOutputDutyCycle:" + mHatchArmRotationMotor.getMCOutputPercent() + ";" +
				"HatchArmOutputVoltage:" + mHatchArmRotationMotor.getMCOutputPercent() * mHatchArmRotationMotor.getMCInputVoltage() + ";" +
				"HatchArmSupplyVoltage:" + mHatchArmRotationMotor.getMCInputVoltage() + ";" +
				"HatchArmRotationMotorHasReset:" + mHatchArmRotationMotor.hasMotorControllerReset().getMessage() + ";" +
				"HatchArmRollerMotorHasReset:" + mHatchArmRollerMotor.hasMotorControllerReset().getMessage() + ";" +
				"HatchArmControlMode:" + mHatchArmControlMode.toString() + ";" +
				"HatchArmIntakeCurrent:" + mHatchArmRollerMotor.getMCOutputCurrent() + ";" +
				"HatchArmIntakeOutputDutyCycle:" + mHatchArmRollerMotor.getMCOutputPercent() + ";" +
				"HatchArmIntakeOutputVoltage:" + mHatchArmRollerMotor.getMCOutputPercent() * mHatchArmRollerMotor.getMCInputVoltage() + ";" +
				"HatchArmIntakeSupplyVoltage:" + mHatchArmRollerMotor.getMCInputVoltage() + ";";
	}

	@Override
	public void zeroSensors() {
		mHatchArmRotationMotor.setEncoderPosition(0);
	}

	@Override
	public void registerEnabledLoops(ILooper in) {
		in.register(mLoop);
	}

	private final Loop mLoop = new Loop() {
		@Override
		public void onFirstStart(double timestamp) {
			synchronized (HatchIntakeArm.this) {
				zeroSensors();
			}
		}

		@Override
		public void onStart(double timestamp) {
			synchronized (HatchIntakeArm.this) {

			}
		}

		@SuppressWarnings("Duplicates")
		@Override
		public void onLoop(double timestamp) {
			synchronized (HatchIntakeArm.this) {
				switch (mHatchArmControlMode) {
					case POSITION:
						if (hatchArmAnyPositionCheck.hasPassedConditions() && hatchArmAnyPositionCheck.isEnabled())
							mHatchArmRotationMotor.set(MCControlMode.MotionMagic, mHatchArmSetpoint, 0, 0);
						else if (hatchArmPauseDownCheck.hasPassedConditions() && hatchArmPauseDownCheck.isEnabled())
							mHatchArmRotationMotor.set(MCControlMode.MotionMagic, Math.max(mHatchArmSetpoint, HatchArmPositions.CollisionThreshold + HatchArmPositions.PositionDelta), 0, 0);
						else if (hatchArmPauseUpCheck.hasPassedConditions() && hatchArmPauseUpCheck.isEnabled())
							mHatchArmRotationMotor.set(MCControlMode.MotionMagic, Math.min(mHatchArmSetpoint, HatchArmPositions.Inside), 0, 0);
						else
							mHatchArmRotationMotor.set(MCControlMode.MotionMagic, mHatchArmSetpoint, 0, 0);
						break;
					case OPEN_LOOP:
						mHatchArmRotationMotor.set(MCControlMode.PercentOut, Math.min(Math.max(mHatchArmSetpoint, -1), 1), 0, 0);
						break;
					case DISABLED:
					default:
						mHatchArmRotationMotor.set(MCControlMode.Disabled, 0, 0, 0);
						break;
				}

				mHatchArmRollerMotor.set(MCControlMode.PercentOut, Math.min(Math.max(mHatchRollerSetpoint, -1), 1), 0, 0);
			}
		}

		@Override
		public void onStop(double timestamp) {
			stop();
		}
	};

	public void setCollisionAvoidanceEnabled(boolean enabled) {
		hatchArmAnyPositionCheck.setEnabled(enabled);
		hatchArmPauseDownCheck.setEnabled(enabled);
		hatchArmPauseUpCheck.setEnabled(enabled);
	}

	@Override
	public double getPosition() {
		return mHatchArmRotationMotor.getPosition();
	}

	@Override
	public double getSetpoint() {
		return mHatchArmSetpoint;
	}

	public boolean isArmAtSetpoint(double posDelta) {
		return Math.abs(mHatchArmSetpoint - mHatchArmRotationMotor.getPosition()) < Math.abs(posDelta);
	}

	public synchronized void setHatchArmPosition(double armPosition) {
		mHatchArmSetpoint = armPosition;
	}

	public synchronized void setHatchRollerSpeed(double rollerSpeed) {
		mHatchRollerSetpoint = rollerSpeed;
	}

	private synchronized void setHatchArmControlMode(HatchArmControlMode hatchArmControlMode) {
		mHatchArmControlMode = hatchArmControlMode;
	}

	public enum HatchArmControlMode {
		POSITION,
		OPEN_LOOP,
		DISABLED;
	}
}