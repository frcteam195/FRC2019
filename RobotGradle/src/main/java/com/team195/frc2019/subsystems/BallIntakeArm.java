package com.team195.frc2019.subsystems;

import com.team195.frc2019.Constants;
import com.team195.frc2019.loops.ILooper;
import com.team195.frc2019.loops.Loop;
import com.team195.lib.drivers.CKSolenoid;
import com.team195.lib.drivers.motorcontrol.CKTalonSRX;
import com.team195.lib.drivers.motorcontrol.MCControlMode;
import com.team195.lib.drivers.motorcontrol.PDPBreaker;
import com.team195.lib.util.InterferenceSystem;
import com.team195.lib.util.MotionInterferenceChecker;

public class BallIntakeArm extends Subsystem implements InterferenceSystem {

	private static BallIntakeArm mInstance = new BallIntakeArm();

	private final CKTalonSRX mBallArmRotationMotor;
	private final CKTalonSRX mBallArmRollerMotor;

	private final MotionInterferenceChecker ballArmUpCheck;

	private final CKSolenoid mBallIntakeBarDropSolenoid;

	private BallIntakeArmControlMode mBallIntakeArmControlMode = BallIntakeArmControlMode.POSITION;

	private double mBallIntakeArmSetpoint = 0;

	private BallIntakeArm() {
		mBallArmRotationMotor = new CKTalonSRX(Constants.kBallIntakeRotationMotorId, false, PDPBreaker.B30A);
		mBallArmRollerMotor = new CKTalonSRX(Constants.kBallIntakeRollerMotorId, false, PDPBreaker.B30A);

		ballArmUpCheck = new MotionInterferenceChecker(
				(t) -> Elevator.getInstance().getPosition() > Constants.kElevatorPosToBallIntakeArm
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
		if (systemFaulted)
			setBallIntakeArmControlMode(BallIntakeArmControlMode.OPEN_LOOP);
		return systemFaulted;
	}

	@Override
	public boolean runDiagnostics() {
		return false;
	}

	@Override
	public String generateReport() {
		String retVal = "";
		retVal += "BallArmPos:" + mBallArmRotationMotor.getVelocity() + ";";
		retVal += "BallArmVel:" + mBallArmRotationMotor.getVelocity() + ";";
		retVal += "BallArmOutput:" + mBallIntakeArmSetpoint + ";";
		retVal += "BallArmCurrent:" + mBallArmRotationMotor.getMCOutputCurrent() + ";";
		retVal += "BallArmOutputDutyCycle:" + mBallArmRotationMotor.getMCOutputPercent() + ";";
		retVal += "BallArmOutputVoltage:" + mBallArmRotationMotor.getMCOutputPercent()*mBallArmRotationMotor.getMCInputVoltage() + ";";
		retVal += "BallArmSupplyVoltage:" + mBallArmRotationMotor.getMCInputVoltage() + ";";
		retVal += "BallArmControlMode:" + mBallIntakeArmControlMode.toString() + ";";
		retVal += "BallArmIntakeCurrent:" + mBallArmRollerMotor.getMCOutputCurrent() + ";";
		retVal += "BallArmIntakeOutputDutyCycle:" + mBallArmRollerMotor.getMCOutputPercent() + ";";
		retVal += "BallArmIntakeOutputVoltage:" + mBallArmRollerMotor.getMCOutputPercent()*mBallArmRollerMotor.getMCInputVoltage() + ";";
		retVal += "BallArmIntakeSupplyVoltage:" + mBallArmRollerMotor.getMCInputVoltage() + ";";
		retVal += "BallArmIntakeControlMode:" + mBallArmRollerMotor.toString() + ";";
		return retVal;
	}

	@Override
	public void zeroSensors() {
		mBallArmRotationMotor.setEncoderPosition(0);
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
						if (mBallIntakeArmSetpoint > Constants.kBallIntakeArmPosToElevator && !ballArmUpCheck.hasPassedConditions())
							mBallArmRotationMotor.set(MCControlMode.MotionMagic, Constants.kBallIntakeArmPosToElevator, 0, 0);
						else
							mBallArmRotationMotor.set(MCControlMode.MotionMagic, mBallIntakeArmSetpoint, 0, 0);
						break;
					case OPEN_LOOP:
						mBallArmRotationMotor.set(MCControlMode.PercentOut, Math.min(Math.max(mBallIntakeArmSetpoint, -1), 1), 0, 0);
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
		return mBallArmRotationMotor.getPosition();
	}

	public synchronized void setBallIntakeArmPosition(double armPosition) {
		mBallIntakeArmSetpoint = armPosition;
	}

	private synchronized void setBallIntakeArmControlMode(BallIntakeArmControlMode ballIntakeArmControlMode) {
		mBallIntakeArmControlMode = ballIntakeArmControlMode;
	}

	public enum BallIntakeArmControlMode {
		POSITION,
		OPEN_LOOP;
	}
}
