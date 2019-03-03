package com.team195.frc2019.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team195.frc2019.Constants;
import com.team195.frc2019.loops.ILooper;
import com.team195.frc2019.loops.Loop;
import com.team195.lib.drivers.motorcontrol.CKTalonSRX;
import com.team195.lib.drivers.motorcontrol.MCControlMode;
import com.team195.lib.drivers.motorcontrol.PDPBreaker;
import com.team195.lib.drivers.motorcontrol.TuneablePIDOSC;
import com.team195.lib.util.InterferenceSystem;
import com.team195.lib.util.MotionInterferenceChecker;

public class HatchIntakeArm extends Subsystem implements InterferenceSystem {

	private static HatchIntakeArm mInstance = new HatchIntakeArm();

	private final CKTalonSRX mHatchArmRotationMotor;
	private final CKTalonSRX mHatchArmRollerMotor;

	private final MotionInterferenceChecker hatchArmUpCheck;

	private HatchArmControlMode mHatchArmControlMode = HatchArmControlMode.POSITION;

	private double mHatchArmSetpoint = 0;

	private HatchIntakeArm() {
		mHatchArmRotationMotor = new CKTalonSRX(Constants.kHatchIntakeRotationMotorId, false, PDPBreaker.B30A);
		mHatchArmRotationMotor.setInverted(true);
		mHatchArmRotationMotor.setSensorPhase(true);
		mHatchArmRotationMotor.setPIDF(Constants.kHatchArmPositionKp, Constants.kHatchArmPositionKi, Constants.kHatchArmPositionKd, Constants.kHatchArmPositionKf);
		mHatchArmRotationMotor.setMotionParameters(Constants.kHatchArmPositionCruiseVel, Constants.kHatchArmPositionMMAccel);
		mHatchArmRotationMotor.setControlMode(MCControlMode.MotionMagic);

//		TuneablePIDOSC x;
//		try {
//			x = new TuneablePIDOSC("Hatch Arm", 5804, true, mHatchArmRotationMotor);
//		} catch (Exception ignored) {
//
//		}

		mHatchArmRollerMotor = new CKTalonSRX(Constants.kHatchIntakeRollerMotorId, false, PDPBreaker.B30A);

		hatchArmUpCheck = new MotionInterferenceChecker(
				(t) -> Elevator.getInstance().getPosition() > Constants.kElevatorPosToHatchIntakeArm
		);
	}

	public static HatchIntakeArm getInstance() {
		return mInstance;
	}

	@Override
	public void stop() {

	}

	@Override
	public boolean isSystemFaulted() {
		boolean systemFaulted = !mHatchArmRotationMotor.isEncoderPresent();
		if (systemFaulted)
			setHatchArmControlMode(HatchArmControlMode.OPEN_LOOP);
		return systemFaulted;
	}

	@Override
	public boolean runDiagnostics() {
		return false;
	}

	@Override
	public String generateReport() {
		String retVal = "";
		retVal += "HatchArmPos:" + mHatchArmRotationMotor.getVelocity() + ";";
		retVal += "HatchArmVel:" + mHatchArmRotationMotor.getVelocity() + ";";
		retVal += "HatchArmOutput:" + mHatchArmSetpoint + ";";
		retVal += "HatchArmCurrent:" + mHatchArmRotationMotor.getMCOutputCurrent() + ";";
		retVal += "HatchArmOutputDutyCycle:" + mHatchArmRotationMotor.getMCOutputPercent() + ";";
		retVal += "HatchArmOutputVoltage:" + mHatchArmRotationMotor.getMCOutputPercent()*mHatchArmRotationMotor.getMCInputVoltage() + ";";
		retVal += "HatchArmSupplyVoltage:" + mHatchArmRotationMotor.getMCInputVoltage() + ";";
		retVal += "HatchArmControlMode:" + mHatchArmControlMode.toString() + ";";
		retVal += "HatchArmIntakeCurrent:" + mHatchArmRollerMotor.getMCOutputCurrent() + ";";
		retVal += "HatchArmIntakeOutputDutyCycle:" + mHatchArmRollerMotor.getMCOutputPercent() + ";";
		retVal += "HatchArmIntakeOutputVoltage:" + mHatchArmRollerMotor.getMCOutputPercent()*mHatchArmRollerMotor.getMCInputVoltage() + ";";
		retVal += "HatchArmIntakeSupplyVoltage:" + mHatchArmRollerMotor.getMCInputVoltage() + ";";
//		retVal += "HatchArmIntakeControlMode:" + mHatchIntakeArmControlMode.toString() + ";";
		return retVal;
	}

	@Override
	public void zeroSensors() {
		mHatchArmRotationMotor.setEncoderPosition(0);
		if (mHatchArmControlMode == HatchArmControlMode.POSITION)
			mHatchArmRotationMotor.set(MCControlMode.MotionMagic, 0, 0, 0);
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
//						if (mHatchArmSetpoint > Constants.kHatchIntakeArmPosToElevator && !hatchArmUpCheck.hasPassedConditions())
//							mHatchArmRotationMotor.set(MCControlMode.MotionMagic, Constants.kHatchIntakeArmPosToElevator, 0, 0);
//						else
//							mHatchArmRotationMotor.set(MCControlMode.MotionMagic, mHatchArmSetpoint, 0, 0);
						break;
					case OPEN_LOOP:
						mHatchArmRotationMotor.set(MCControlMode.PercentOut, Math.min(Math.max(mHatchArmSetpoint, -1), 1), 0, 0);
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
	};

	@Override
	public double getPosition() {
		return mHatchArmRotationMotor.getPosition();
	}

	public synchronized void setHatchArmPosition(double armPosition) {
		mHatchArmSetpoint = armPosition;
	}

	private synchronized void setHatchArmControlMode(HatchArmControlMode hatchArmControlMode) {
		mHatchArmControlMode = hatchArmControlMode;
	}

	public enum HatchArmControlMode {
		POSITION,
		OPEN_LOOP;
	}
}