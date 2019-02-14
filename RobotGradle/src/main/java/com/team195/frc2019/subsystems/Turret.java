package com.team195.frc2019.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.team195.frc2019.Constants;
import com.team195.frc2019.auto.actions.AutomatedActions;
import com.team195.frc2019.loops.ILooper;
import com.team195.frc2019.loops.Loop;
import com.team195.lib.drivers.CKDoubleSolenoid;
import com.team195.lib.drivers.motorcontrol.CKTalonSRX;
import com.team195.lib.drivers.motorcontrol.MCControlMode;
import com.team195.lib.drivers.motorcontrol.PDPBreaker;
import com.team195.lib.util.TeleopActionRunner;

public class Turret extends Subsystem {

	private static Turret mInstance = new Turret();

	private final CKTalonSRX mTurretRotationMotor;
	private final CKTalonSRX mTurretRollerMotor;
	private final CKDoubleSolenoid mHatchBeakSolenoid;
	private final CKDoubleSolenoid mHatchPushSolenoid;

	private TurretControlMode mTurretControlMode = TurretControlMode.POSITION;

	private double mTurretSetpoint = 0;

	private TeleopActionRunner mAutoHatchController = null;

	private Turret() {
		mTurretRotationMotor = new CKTalonSRX(Constants.kTurretMotorId, false, PDPBreaker.B30A);
		mTurretRollerMotor = new CKTalonSRX(Constants.kBallShooterMotorId, false, PDPBreaker.B30A);
		mTurretRollerMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
		mHatchBeakSolenoid = new CKDoubleSolenoid(Constants.kHatchBeakSolenoidId);
		mHatchBeakSolenoid.configReversed(false);
		mHatchBeakSolenoid.set(false);
		mHatchPushSolenoid = new CKDoubleSolenoid(Constants.kHatchPushSolenoidId);
		mHatchPushSolenoid.configReversed(false);
		mHatchPushSolenoid.set(false);
	}

	public static Turret getInstance() {
		return mInstance;
	}

	@Override
	public void stop() {

	}

	@Override
	public boolean isSystemFaulted() {
		boolean systemFaulted = !mTurretRotationMotor.isEncoderPresent();
		if (systemFaulted)
			setTurretControlMode(TurretControlMode.OPEN_LOOP);
		return systemFaulted;
	}

	@Override
	public boolean runDiagnostics() {
		return false;
	}

	@Override
	public String generateReport() {
		String retVal = "";
		retVal += "TurretPos:" + mTurretRotationMotor.getVelocity() + ";";
		retVal += "TurretVel:" + mTurretRotationMotor.getVelocity() + ";";
		retVal += "TurretOutput:" + mTurretSetpoint + ";";
		retVal += "TurretCurrent:" + mTurretRotationMotor.getMCOutputCurrent() + ";";
		retVal += "TurretOutputDutyCycle:" + mTurretRotationMotor.getMCOutputPercent() + ";";
		retVal += "TurretOutputVoltage:" + mTurretRotationMotor.getMCOutputPercent()* mTurretRotationMotor.getMCInputVoltage() + ";";
		retVal += "TurretSupplyVoltage:" + mTurretRotationMotor.getMCInputVoltage() + ";";
		retVal += "TurretControlMode:" + mTurretControlMode.toString() + ";";
		retVal += "TurretIntakeCurrent:" + mTurretRollerMotor.getMCOutputCurrent() + ";";
		retVal += "TurretIntakeOutputDutyCycle:" + mTurretRollerMotor.getMCOutputPercent() + ";";
		retVal += "TurretIntakeOutputVoltage:" + mTurretRollerMotor.getMCOutputPercent()* mTurretRollerMotor.getMCInputVoltage() + ";";
		retVal += "TurretIntakeSupplyVoltage:" + mTurretRollerMotor.getMCInputVoltage() + ";";
		retVal += "TurretIntakeControlMode:" + mTurretRollerMotor.toString() + ";";
		return retVal;
	}

	@Override
	public void zeroSensors() {
		mTurretRotationMotor.setEncoderPosition(0);
	}

	@Override
	public void registerEnabledLoops(ILooper in) {
		in.register(mLoop);
	}

	private final Loop mLoop = new Loop() {
		@Override
		public void onFirstStart(double timestamp) {
			synchronized (Turret.this) {
				zeroSensors();
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
					case POSITION:
						mTurretRotationMotor.set(MCControlMode.MotionMagic, mTurretSetpoint, 0, 0);
						break;
					case OPEN_LOOP:
						mTurretRotationMotor.set(MCControlMode.PercentOut, Math.min(Math.max(mTurretSetpoint, -1), 1), 0, 0);
						break;
					default:
						break;
				}

				if (mAutoHatchController == null && mTurretRollerMotor.getForwardLimitRisingEdge()) {
					mAutoHatchController = new TeleopActionRunner(AutomatedActions.pushOutHatch());
					mAutoHatchController.runAction(false);
				}

				if (mAutoHatchController.isFinished())
					mAutoHatchController = null;
			}
		}

		@Override
		public void onStop(double timestamp) {
			stop();
		}
	};

	public synchronized void setHatchPush(boolean hatchPush) {
		mHatchPushSolenoid.set(hatchPush);
	}

	public synchronized void setBeak(boolean open) {
		mHatchBeakSolenoid.set(open);
	}

	public synchronized void setTurretPosition(double turretPosition) {
		mTurretSetpoint = turretPosition;
	}

	private synchronized void setTurretControlMode(TurretControlMode turretControlMode) {
		mTurretControlMode = turretControlMode;
	}

	public enum TurretControlMode {
		POSITION,
		OPEN_LOOP;
	}
}