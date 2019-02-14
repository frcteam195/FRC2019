package com.team195.frc2019.subsystems;

import com.team195.frc2019.Constants;
import com.team195.frc2019.loops.ILooper;
import com.team195.frc2019.loops.Loop;
import com.team195.lib.drivers.motorcontrol.CKTalonSRX;
import com.team195.lib.drivers.motorcontrol.MCControlMode;
import com.team195.lib.drivers.motorcontrol.PDPBreaker;
import com.team195.lib.util.InterferenceSystem;
import com.team195.lib.util.MotionInterferenceChecker;

public class HatchIntakeArm extends Subsystem implements InterferenceSystem {

	private static HatchIntakeArm mInstance = new HatchIntakeArm();

	private final CKTalonSRX mHatchArmRotationMotor;
	private final CKTalonSRX mHatchArmRollerMotor;

	private final MotionInterferenceChecker hatchArmUpCheck;

	private HatchIntakeArmControlMode mHatchIntakeArmControlMode = HatchIntakeArmControlMode.POSITION;

	private double mHatchIntakeArmSetpoint = 0;

	public HatchIntakeArm() {
		mHatchArmRotationMotor = new CKTalonSRX(Constants.kHatchIntakeRotationMotorId, false, PDPBreaker.B30A);
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
			setHatchIntakeArmControlMode(HatchIntakeArmControlMode.OPEN_LOOP);
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
		retVal += "HatchArmOutput:" + mHatchIntakeArmSetpoint + ";";
		retVal += "HatchArmCurrent:" + mHatchArmRotationMotor.getMCOutputCurrent() + ";";
		retVal += "HatchArmOutputDutyCycle:" + mHatchArmRotationMotor.getMCOutputPercent() + ";";
		retVal += "HatchArmOutputVoltage:" + mHatchArmRotationMotor.getMCOutputPercent()*mHatchArmRotationMotor.getMCInputVoltage() + ";";
		retVal += "HatchArmSupplyVoltage:" + mHatchArmRotationMotor.getMCInputVoltage() + ";";
		retVal += "HatchArmControlMode:" + mHatchIntakeArmControlMode.toString() + ";";
		retVal += "HatchArmIntakeCurrent:" + mHatchArmRollerMotor.getMCOutputCurrent() + ";";
		retVal += "HatchArmIntakeOutputDutyCycle:" + mHatchArmRollerMotor.getMCOutputPercent() + ";";
		retVal += "HatchArmIntakeOutputVoltage:" + mHatchArmRollerMotor.getMCOutputPercent()*mHatchArmRollerMotor.getMCInputVoltage() + ";";
		retVal += "HatchArmIntakeSupplyVoltage:" + mHatchArmRollerMotor.getMCInputVoltage() + ";";
		retVal += "HatchArmIntakeControlMode:" + mHatchArmRollerMotor.toString() + ";";
		return retVal;
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
				switch (mHatchIntakeArmControlMode) {
					case POSITION:
						if (mHatchIntakeArmSetpoint > Constants.kHatchIntakeArmPosToElevator && !hatchArmUpCheck.hasPassedConditions())
							mHatchArmRotationMotor.set(MCControlMode.MotionMagic, Constants.kHatchIntakeArmPosToElevator, 0, 0);
						else
							mHatchArmRotationMotor.set(MCControlMode.MotionMagic, mHatchIntakeArmSetpoint, 0, 0);
						break;
					case OPEN_LOOP:
						mHatchArmRotationMotor.set(MCControlMode.PercentOut, Math.min(Math.max(mHatchIntakeArmSetpoint, -1), 1), 0, 0);
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

	public synchronized void setHatchIntakeArmPosition(double armPosition) {
		mHatchIntakeArmSetpoint = armPosition;
	}

	private synchronized void setHatchIntakeArmControlMode(HatchIntakeArmControlMode hatchIntakeArmControlMode) {
		mHatchIntakeArmControlMode = hatchIntakeArmControlMode;
	}

	public enum HatchIntakeArmControlMode {
		POSITION,
		OPEN_LOOP;
	}
}