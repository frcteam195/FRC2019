package com.team195.frc2019.subsystems;

import com.team195.frc2019.Constants;
import com.team195.frc2019.loops.ILooper;
import com.team195.frc2019.loops.Loop;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.subsystems.positions.BallIntakeArmPositions;
import com.team195.frc2019.subsystems.positions.ElevatorPositions;
import com.team195.frc2019.subsystems.positions.HatchArmPositions;
import com.team195.frc2019.subsystems.positions.TurretPositions;
import com.team195.lib.drivers.motorcontrol.CKTalonSRX;
import com.team195.lib.drivers.motorcontrol.MCControlMode;
import com.team195.lib.drivers.motorcontrol.PDPBreaker;
import com.team195.lib.drivers.motorcontrol.TuneablePIDOSC;
import com.team195.lib.util.InterferenceSystem;
import com.team195.lib.util.MotionInterferenceChecker;

public class Elevator extends Subsystem implements InterferenceSystem {

	private static Elevator mInstance = new Elevator();

	private final CKTalonSRX mElevatorMaster;
	private final CKTalonSRX mElevatorSlaveA;
	private final CKTalonSRX mElevatorSlaveB;
	private final CKTalonSRX mElevatorSlaveC;

//	private final MotionInterferenceChecker elevatorAnyPositionCheck;
	private final MotionInterferenceChecker requestMoveElevatorUpCheck;
	private final MotionInterferenceChecker requestMoveElevatorDownCheck;

	private ElevatorControlMode mElevatorControlMode = ElevatorControlMode.POSITION;

	private double mElevatorSetpoint = 0;

	private Elevator() {
		mElevatorMaster = new CKTalonSRX(Constants.kElevatorMasterLeftId, false, PDPBreaker.B40A);
		mElevatorMaster.setSensorPhase(true);
		mElevatorMaster.setInverted(false);
		mElevatorMaster.setPIDF(Constants.kElevatorPositionKp, Constants.kElevatorPositionKi, Constants.kElevatorPositionKd, Constants.kElevatorPositionKf);
		mElevatorMaster.setMotionParameters(Constants.kElevatorPositionCruiseVel, Constants.kElevatorPositionMMAccel);
		zeroSensors();
		mElevatorMaster.setControlMode(MCControlMode.Disabled);
		mElevatorMaster.configForwardSoftLimitThreshold(Constants.kElevatorPositionForwardSoftLimit);
		mElevatorMaster.configForwardSoftLimitEnable(true);
		mElevatorMaster.configReverseSoftLimitThreshold(Constants.kElevatorPositionReverseSoftLimit);
		mElevatorMaster.configReverseSoftLimitEnable(true);

//		TuneablePIDOSC x;
//		try {
//			x = new TuneablePIDOSC("Elevator", 5804, true, mElevatorMaster);
//		} catch (Exception ignored) {
//
//		}

		mElevatorSlaveA = new CKTalonSRX(Constants.kElevatorSlaveALeftId, mElevatorMaster, PDPBreaker.B40A, false);

		mElevatorSlaveB = new CKTalonSRX(Constants.kElevatorSlaveBRightId, mElevatorMaster, PDPBreaker.B40A, true);
		mElevatorSlaveC = new CKTalonSRX(Constants.kElevatorSlaveCRightId, mElevatorMaster, PDPBreaker.B40A, true);

		requestMoveElevatorUpCheck = new MotionInterferenceChecker(MotionInterferenceChecker.LogicOperation.OR, true,
				(t) -> ((HatchIntakeArm.getInstance().getSetpoint() == HatchArmPositions.Outside
						&& HatchIntakeArm.getInstance().getPosition() < HatchArmPositions.CollisionThreshold)),
				(t) -> ((HatchIntakeArm.getInstance().getSetpoint() == HatchArmPositions.Inside
						&& Math.abs(HatchIntakeArm.getInstance().getPosition() - HatchArmPositions.Inside) > HatchArmPositions.PositionDelta))
		);

		requestMoveElevatorDownCheck = new MotionInterferenceChecker(MotionInterferenceChecker.LogicOperation.OR, true,
				(t) -> (BallIntakeArm.getInstance().getPosition() > BallIntakeArmPositions.CollisionThreshold - BallIntakeArmPositions.PositionDelta),
				(t) -> (BallIntakeArm.getInstance().getSetpoint() == BallIntakeArmPositions.Up)
		);

	}

	public static Elevator getInstance() {
		return mInstance;
	}

	@Override
	public void stop() {
		mElevatorMaster.set(MCControlMode.PercentOut, 0, 0, 0);
	}

	@Override
	public boolean isSystemFaulted() {
		boolean systemFaulted = !mElevatorMaster.isEncoderPresent();
		if (systemFaulted)
			setElevatorControlMode(ElevatorControlMode.OPEN_LOOP);
		return systemFaulted;
	}

	@Override
	public boolean runDiagnostics() {
		return false;
	}

	@Override
	public String generateReport() {
		return  "ElevatorPos:" + mElevatorMaster.getVelocity() + ";" +
				"ElevatorVel:" + mElevatorMaster.getVelocity() + ";" +
				"ElevatorOutput:" + mElevatorSetpoint + ";" +
				"Elevator1Current:" + mElevatorMaster.getMCOutputCurrent() + ";" +
				"Elevator2Current:" + mElevatorSlaveA.getMCOutputCurrent() + ";" +
				"Elevator3Current:" + mElevatorSlaveB.getMCOutputCurrent() + ";" +
				"ElevatorOutputDutyCycle:" + mElevatorMaster.getMCOutputPercent() + ";" +
				"ElevatorOutputVoltage:" + mElevatorMaster.getMCOutputPercent() * mElevatorMaster.getMCInputVoltage() + ";" +
				"ElevatorSupplyVoltage:" + mElevatorMaster.getMCInputVoltage() + ";" +
				"ElevatorControlMode:" + mElevatorControlMode.toString() + ";";
	}

	@Override
	public void zeroSensors() {
		mElevatorMaster.setEncoderPosition(0);
	}

	@Override
	public void registerEnabledLoops(ILooper in) {
		in.register(mLoop);
	}

	private final Loop mLoop = new Loop() {
		@Override
		public void onFirstStart(double timestamp) {
			synchronized (Elevator.this) {
				zeroSensors();
			}
		}

		@Override
		public void onStart(double timestamp) {
			synchronized (Elevator.this) {

			}
		}

		@SuppressWarnings("Duplicates")
		@Override
		public void onLoop(double timestamp) {
			synchronized (Elevator.this) {
				switch (mElevatorControlMode) {
					case POSITION:
						double outputPos = mElevatorSetpoint;

						boolean eUp = requestMoveElevatorUpCheck.hasPassedConditions() && requestMoveElevatorUpCheck.isEnabled();
						boolean eDown = requestMoveElevatorDownCheck.hasPassedConditions() && requestMoveElevatorDownCheck.isEnabled();

						if (eUp) {
							outputPos = Math.max(outputPos, ElevatorPositions.CollisionThresholdHatchArm + ElevatorPositions.PositionDelta);
						}
						if (eDown) {
							outputPos = Math.min(outputPos, ElevatorPositions.CollisionThresholdBallArm - ElevatorPositions.PositionDelta);
						}

						if (eUp || eDown || (!eUp && !eDown))
							mElevatorMaster.set(MCControlMode.MotionMagic, outputPos, 0, 0);

						break;
					case OPEN_LOOP:
						mElevatorMaster.set(MCControlMode.PercentOut, Math.min(Math.max(mElevatorSetpoint, -1), 1), 0, 0);
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

	public void setCollisionAvoidanceEnabled(boolean enabled) {
		requestMoveElevatorDownCheck.setEnabled(enabled);
		requestMoveElevatorUpCheck.setEnabled(enabled);
	}

	@Override
	public double getPosition() {
		return mElevatorMaster.getPosition();
	}

	@Override
	public double getSetpoint() {
		return mElevatorSetpoint;
	}

	public synchronized void setElevatorPosition(double elevatorPosition) {
		mElevatorSetpoint = elevatorPosition;
	}

	public boolean isElevatorAtSetpoint(double posDelta) {
		return Math.abs(mElevatorSetpoint - mElevatorMaster.getPosition()) < Math.abs(posDelta);
	}

	private synchronized void setElevatorControlMode (ElevatorControlMode elevatorControlMode) {
		mElevatorControlMode = elevatorControlMode;
	}

	public enum ElevatorControlMode {
		POSITION,
		OPEN_LOOP;
	}
}
