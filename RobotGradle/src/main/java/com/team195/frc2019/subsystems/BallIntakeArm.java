package com.team195.frc2019.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
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

	private BallIntakeArmControlMode mBallIntakeArmControlMode = BallIntakeArmControlMode.OPEN_LOOP;

	private double mBallIntakeArmSetpoint = 0;
	private double mBallIntakeRollerSetpoint = 0;

	private BallIntakeArm() {
		mBallArmRotationMotor = new CKTalonSRX(Constants.kBallIntakeRotationMotorId, false, PDPBreaker.B30A);
		mBallArmRotationMotor.setSensorPhase(true);
		mBallArmRotationMotor.setPIDGainSlot(0);
		mBallArmRotationMotor.setFeedbackDevice(FeedbackDevice.CTRE_MagEncoder_Relative);
		mBallArmRotationMotor.setPIDF(Constants.kBallIntakeArmUpPositionKp, Constants.kBallIntakeArmUpPositionKi, Constants.kBallIntakeArmUpPositionKd, Constants.kBallIntakeArmUpPositionKf);
		mBallArmRotationMotor.setMotionParameters(Constants.kBallIntakeArmUpPositionCruiseVel, Constants.kBallIntakeArmUpPositionMMAccel);
		mBallArmRotationMotor.setPIDGainSlot(1);
		mBallArmRotationMotor.setFeedbackDevice(RemoteFeedbackDevice.RemoteSensor0, Constants.kBallIntakeRollerMotorId);
		mBallArmRotationMotor.setPIDF(Constants.kBallIntakeArmDownPositionKp, Constants.kBallIntakeArmDownPositionKi, Constants.kBallIntakeArmDownPositionKd, Constants.kBallIntakeArmDownPositionKf);
		mBallArmRotationMotor.setMotionParameters(Constants.kBallIntakeArmDownPositionCruiseVel, Constants.kBallIntakeArmDownPositionMMAccel);
		zeroSensors();
		mBallArmRotationMotor.configForwardSoftLimitThreshold(Constants.kBallIntakeArmForwardSoftLimit);
		mBallArmRotationMotor.configForwardSoftLimitEnable(true);
		mBallArmRotationMotor.configReverseSoftLimitEnable(false);
		mBallArmRotationMotor.setControlMode(MCControlMode.MotionMagic);

//		TuneablePIDOSC x;
//		try {
//			x = new TuneablePIDOSC("Ball Arm", 5804, true, mBallArmRotationMotor);
//		} catch (Exception ignored) {
//
//		}

		mBallArmRollerMotor = new CKTalonSRX(Constants.kBallIntakeRollerMotorId, false, PDPBreaker.B30A);
		mBallArmRollerMotor.setInverted(true);

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
//		retVal += "BallArmIntakeControlMode:" + mBallIntakeArmControlMode.toString() + ";";
		return retVal;
	}

	@Override
	public void zeroSensors() {
		mBallArmRotationMotor.setEncoderPosition(0);
		mBallArmRollerMotor.setEncoderPosition(0);
		if (mBallIntakeArmControlMode == BallIntakeArmControlMode.POSITION)
			mBallArmRotationMotor.set(MCControlMode.MotionMagic, 0, 0, 0);
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
//						if (mBallIntakeArmSetpoint > Constants.kBallIntakeArmPosToElevator && !ballArmUpCheck.hasPassedConditions())
//							mBallArmRotationMotor.set(MCControlMode.MotionMagic, Constants.kBallIntakeArmPosToElevator, 0, 0);
//						else
//							mBallArmRotationMotor.set(MCControlMode.MotionMagic, mBallIntakeArmSetpoint, 0, 0);
//
						int rotationPIDSlot = 0;
						if (mBallIntakeArmSetpoint == 0)
							rotationPIDSlot = 1;

						mBallArmRotationMotor.set(MCControlMode.MotionMagic, mBallIntakeArmSetpoint, rotationPIDSlot, 0);
						break;
					case OPEN_LOOP:
						mBallArmRotationMotor.set(MCControlMode.PercentOut, Math.min(Math.max(mBallIntakeArmSetpoint, -1), 1), 0, 0);
						break;
					case DISABLED:
					default:
						mBallArmRotationMotor.set(MCControlMode.Disabled, 0, 0, 0);
						break;
				}

				mBallArmRollerMotor.set(MCControlMode.PercentOut, Math.min(Math.max(mBallIntakeRollerSetpoint, -1), 1), 0, 0);
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
