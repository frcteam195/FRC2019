package com.team195.frc2019.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.team195.frc2019.Constants;
import com.team195.frc2019.auto.actions.AutomatedActions;
import com.team195.frc2019.loops.ILooper;
import com.team195.frc2019.loops.Loop;
import com.team195.lib.drivers.CKDoubleSolenoid;
import com.team195.lib.drivers.CKSolenoid;
import com.team195.lib.drivers.motorcontrol.CKTalonSRX;
import com.team195.lib.drivers.motorcontrol.MCControlMode;
import com.team195.lib.drivers.motorcontrol.PDPBreaker;
import com.team195.lib.drivers.motorcontrol.TuneablePIDOSC;
import com.team195.lib.util.TeleopActionRunner;

public class Turret extends Subsystem {

	private static Turret mInstance = new Turret();

	private final CKTalonSRX mTurretRotationMotor;
	private final CKTalonSRX mBallShooterRollerMotor;
	private final CKDoubleSolenoid mHatchBeakSolenoid;
	private final CKSolenoid mHatchPushSolenoid;
	private final CKSolenoid mBallPushSolenoid;

	private TurretControlMode mTurretControlMode = TurretControlMode.OPEN_LOOP;
	private BallShooterControlMode mBallShooterControlMode = BallShooterControlMode.OPEN_LOOP;

	private double mTurretSetpoint = 0;
	private double mBallShooterSetpoint = 0;

	private TeleopActionRunner mAutoHatchController = null;

	private Turret() {
		//Encoder on 50:1
		//Turret gear is another 36:254 or 36:252
		mTurretRotationMotor = new CKTalonSRX(Constants.kTurretMotorId, false, PDPBreaker.B30A);
		mTurretRotationMotor.setInverted(true);
		mTurretRotationMotor.setSensorPhase(true);
		mTurretRotationMotor.setPIDF(Constants.kTurretPositionKp, Constants.kTurretPositionKi, Constants.kTurretPositionKd, Constants.kTurretPositionKf);
		mTurretRotationMotor.setMotionParameters(Constants.kTurretPositionCruiseVel, Constants.kTurretPositionMMAccel);
		zeroSensors();
		mTurretRotationMotor.setControlMode(MCControlMode.MotionMagic);

//		TuneablePIDOSC x;
//		try {
//			x = new TuneablePIDOSC("Turret", 5804, true, mTurretRotationMotor);
//		} catch (Exception ignored) {
//
//		}

		mBallShooterRollerMotor = new CKTalonSRX(Constants.kBallShooterMotorId, false, PDPBreaker.B30A);
		mBallShooterRollerMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

		mHatchBeakSolenoid = new CKDoubleSolenoid(Constants.kHatchBeakSolenoidId);
		mHatchBeakSolenoid.configReversed(false);
		mHatchBeakSolenoid.set(false);

		mHatchPushSolenoid = new CKSolenoid(Constants.kHatchPushSolenoidId);
		mHatchPushSolenoid.set(false);

		mBallPushSolenoid = new CKSolenoid(Constants.kBallPushSolenoidId);
		mBallPushSolenoid.setInverted(false);
		mBallPushSolenoid.set(false);
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
		retVal += "TurretIntakeCurrent:" + mBallShooterRollerMotor.getMCOutputCurrent() + ";";
		retVal += "TurretIntakeOutputDutyCycle:" + mBallShooterRollerMotor.getMCOutputPercent() + ";";
		retVal += "TurretIntakeOutputVoltage:" + mBallShooterRollerMotor.getMCOutputPercent()* mBallShooterRollerMotor.getMCInputVoltage() + ";";
		retVal += "TurretIntakeSupplyVoltage:" + mBallShooterRollerMotor.getMCInputVoltage() + ";";
		retVal += "TurretIntakeControlMode:" + mBallShooterControlMode.toString() + ";";
		return retVal;
	}

	@Override
	public void zeroSensors() {
		mTurretRotationMotor.setEncoderPosition(0);
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
//						mTurretRotationMotor.set(MCControlMode.MotionMagic, mTurretSetpoint, 0, 0);
						break;
					case OPEN_LOOP:
						mTurretRotationMotor.set(MCControlMode.PercentOut, Math.min(Math.max(mTurretSetpoint, -1), 1), 0, 0);
						break;
					default:
						break;
				}

				switch (mBallShooterControlMode) {
					case VELOCITY:
						mBallShooterRollerMotor.set(MCControlMode.SmartVelocity, mBallShooterSetpoint, 0, 0);
						break;
					case CURRENT:
						mBallShooterRollerMotor.set(MCControlMode.Current, mBallShooterSetpoint, 0, 0);
						break;
					case OPEN_LOOP:
						mBallShooterRollerMotor.set(MCControlMode.PercentOut, Math.min(Math.max(mBallShooterSetpoint, -1), 1), 0, 0);
						break;
					default:
						break;
				}

				if (mAutoHatchController == null && mBallShooterRollerMotor.getForwardLimitRisingEdge()) {
					mAutoHatchController = new TeleopActionRunner(AutomatedActions.pushOutHatch());
					mAutoHatchController.runAction(false);
				}

				if (mAutoHatchController != null && mAutoHatchController.isFinished())
					mAutoHatchController = null;
			}
		}

		@Override
		public void onStop(double timestamp) {
			stop();
		}
	};

	public synchronized void setBallPush(boolean ballPush) {
		mBallPushSolenoid.set(ballPush);
	}

	public synchronized void setHatchPush(boolean hatchPush) {
		mHatchPushSolenoid.set(hatchPush);
	}

	public synchronized void setBeak(boolean open) {
		mHatchBeakSolenoid.set(open);
	}

	public synchronized void setBeakOff() {
		mHatchBeakSolenoid.turnOff();
	}

	public synchronized void setTurretPosition(double turretPosition) {
		mTurretSetpoint = turretPosition;
	}

	public synchronized void setBallShooterCurrent(double ballShooterCurrent) {
		setBallShooterControlMode(BallShooterControlMode.CURRENT);
		mBallShooterSetpoint = ballShooterCurrent;
	}

	public synchronized void setBallShooterVelocity(double ballShooterVelocity) {
		setBallShooterControlMode(BallShooterControlMode.VELOCITY);
		mBallShooterSetpoint = ballShooterVelocity;
	}

	public synchronized void setBallShooterOpenLoop(double ballShooterOutput) {
		setBallShooterControlMode(BallShooterControlMode.OPEN_LOOP);
		mBallShooterSetpoint = ballShooterOutput;
	}

	private synchronized void setTurretControlMode(TurretControlMode turretControlMode) {
		if (mTurretControlMode != turretControlMode)
			mTurretControlMode = turretControlMode;
	}

	private synchronized void setBallShooterControlMode(BallShooterControlMode ballShooterControlMode) {
		if (mBallShooterControlMode != ballShooterControlMode)
			mBallShooterControlMode = ballShooterControlMode;
	}

	public boolean isShooterAtSetpoint(double rpmDelta) {
		return Math.abs(mBallShooterSetpoint - mBallShooterRollerMotor.getVelocity()) < Math.abs(rpmDelta);
	}

	public enum TurretControlMode {
		POSITION,
		OPEN_LOOP;
	}

	public enum BallShooterControlMode {
		VELOCITY,
		CURRENT,
		OPEN_LOOP;
	}
}