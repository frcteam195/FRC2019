package com.team195.frc2019.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.team195.frc2019.constants.CalConstants;
import com.team195.frc2019.constants.DeviceIDConstants;
import com.team195.frc2019.constants.TestConstants;
import com.team195.frc2019.loops.ILooper;
import com.team195.frc2019.loops.Loop;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.reporters.DiagnosticMessage;
import com.team195.frc2019.reporters.MessageLevel;
import com.team195.frc2019.reporters.ReflectingLogDataGenerator;
import com.team195.frc2019.subsystems.positions.BallIntakeArmPositions;
import com.team195.frc2019.subsystems.positions.ElevatorPositions;
import com.team195.lib.drivers.motorcontrol.CKTalonSRX;
import com.team195.lib.drivers.motorcontrol.MCControlMode;
import com.team195.lib.drivers.motorcontrol.PDPBreaker;
import com.team195.lib.util.CachedValue;
import com.team195.lib.util.InterferenceSystem;
import com.team195.lib.util.MotionInterferenceChecker;
import com.team195.lib.util.MotorDiagnostics;
import com.team254.lib.util.Util;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class Elevator extends Subsystem implements InterferenceSystem {

	private static Elevator mInstance = new Elevator();

	private PeriodicIO mPeriodicIO;
	private ReflectingLogDataGenerator<PeriodicIO> mLogDataGenerator = new ReflectingLogDataGenerator<>(PeriodicIO.class);

	private final CKTalonSRX mElevatorMaster;
	private final CKTalonSRX mElevatorSlaveA;
	private final CKTalonSRX mElevatorSlaveB;
	private final CKTalonSRX mElevatorSlaveC;

//	private final MotionInterferenceChecker elevatorAnyPositionCheck;
	private final MotionInterferenceChecker requestMoveElevatorUpCheck;
	private final MotionInterferenceChecker requestMoveElevatorDownCheck;

	private ElevatorControlMode mElevatorControlMode = ElevatorControlMode.POSITION;

	private static final int mContinuousCurrentLimit = 15;  //8
	private static final int mPeakCurrentLimit = 20;    //12
	private static final int mPeakCurrentDurationMS = 0;//250;

	private final CachedValue<Boolean> mElevatorEncoderPresent;
	private final CachedValue<Boolean> mElevatorMasterHasReset;

	private Elevator() {
		mPeriodicIO = new PeriodicIO();

		mElevatorMaster = new CKTalonSRX(DeviceIDConstants.kElevatorMasterLeftId, false, PDPBreaker.B40A);
		mElevatorMaster.setSensorPhase(true);
		mElevatorMaster.setInverted(false);
		mElevatorMaster.setPIDF(CalConstants.kElevatorPositionKp, CalConstants.kElevatorPositionKi, CalConstants.kElevatorPositionKd, CalConstants.kElevatorPositionKf);
		mElevatorMaster.setMotionParameters(CalConstants.kElevatorPositionCruiseVel, CalConstants.kElevatorPositionMMAccel);
		zeroSensors();
		mElevatorMaster.setControlMode(MCControlMode.Disabled);
		mElevatorMaster.configForwardSoftLimitThreshold(CalConstants.kElevatorPositionForwardSoftLimit);
		mElevatorMaster.configForwardSoftLimitEnable(true);
		mElevatorMaster.configReverseSoftLimitThreshold(CalConstants.kElevatorPositionReverseSoftLimit);
		mElevatorMaster.configReverseSoftLimitEnable(true);
		mElevatorMaster.configCurrentLimit(mContinuousCurrentLimit, mPeakCurrentLimit, mPeakCurrentDurationMS);


//		TuneablePIDOSC x;
//		try {
//			x = new TuneablePIDOSC("Elevator", 5804, true, mElevatorMaster);
//		} catch (Exception ignored) {
//
//		}

		mElevatorSlaveA = new CKTalonSRX(DeviceIDConstants.kElevatorSlaveALeftId, mElevatorMaster, PDPBreaker.B40A, false);
		mElevatorSlaveA.configCurrentLimit(mContinuousCurrentLimit, mPeakCurrentLimit, mPeakCurrentDurationMS);
		mElevatorSlaveA.setSensorPhase(true);

		mElevatorSlaveB = new CKTalonSRX(DeviceIDConstants.kElevatorSlaveBRightId, mElevatorMaster, PDPBreaker.B40A, true);
		mElevatorSlaveB.configCurrentLimit(mContinuousCurrentLimit, mPeakCurrentLimit, mPeakCurrentDurationMS);
		mElevatorSlaveB.setSensorPhase(true);

		mElevatorSlaveC = new CKTalonSRX(DeviceIDConstants.kElevatorSlaveCRightId, mElevatorMaster, PDPBreaker.B40A, true);
		mElevatorSlaveC.configCurrentLimit(mContinuousCurrentLimit, mPeakCurrentLimit, mPeakCurrentDurationMS);


		//Limit Switch Homing for Elevator
		mElevatorSlaveC.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled);
		mElevatorMaster.configReverseLimitSwitchSource(RemoteLimitSwitchSource.RemoteTalonSRX, LimitSwitchNormal.NormallyOpen, DeviceIDConstants.kElevatorSlaveCRightId);
//		mElevatorMaster.configZeroOnLimit();

		requestMoveElevatorUpCheck = new MotionInterferenceChecker(MotionInterferenceChecker.LogicOperation.OR, true

//				(t) -> ((HatchIntakeArm.getInstance().getSetpoint() == HatchArmPositions.Outside
//						&& HatchIntakeArm.getInstance().getPosition() < HatchArmPositions.CollisionThreshold)),
//				(t) -> ((HatchIntakeArm.getInstance().getSetpoint() == HatchArmPositions.Inside
//						&& Math.abs(HatchIntakeArm.getInstance().getPosition() - HatchArmPositions.Inside) > HatchArmPositions.PositionDelta))
		);

		requestMoveElevatorDownCheck = new MotionInterferenceChecker(MotionInterferenceChecker.LogicOperation.OR, true,
				(t) -> (BallIntakeArm.getInstance().getPosition() > BallIntakeArmPositions.CollisionThreshold - BallIntakeArmPositions.PositionDelta),
				(t) -> (BallIntakeArm.getInstance().getSetpoint() == BallIntakeArmPositions.Up),
				(t) -> (BallIntakeArm.getInstance().isArmUp())
		);

		mElevatorEncoderPresent = new CachedValue<>(500, (t) -> mElevatorMaster.isEncoderPresent());
		mElevatorMasterHasReset = new CachedValue<>(500, (t) -> mElevatorMaster.hasMotorControllerReset() != DiagnosticMessage.NO_MSG);

	}

	public static Elevator getInstance() {
		return mInstance;
	}

	@Override
	public void stop() {
		mElevatorMaster.set(MCControlMode.PercentOut, 0, 0, 0);
	}

	@Override
	public synchronized boolean isSystemFaulted() {
		boolean systemFaulted = !mPeriodicIO.elevator_encoder_present;

		if (systemFaulted) {
			ConsoleReporter.report("Elevator Encoder Missing!", MessageLevel.DEFCON1);
		}

		systemFaulted |= mPeriodicIO.elevator_master_reset;

		if (systemFaulted) {
			ConsoleReporter.report("Elevator Requires Rehoming!", MessageLevel.DEFCON1);
			setElevatorControlMode(ElevatorControlMode.DISABLED);
		}

		return systemFaulted;
	}

	@Override
	public boolean runDiagnostics() {
		if (TestConstants.ENABLE_ELEVATOR_TEST) {
			ConsoleReporter.report("Testing Elevator---------------------------------");
			final double kLowCurrentThres = TestConstants.kElevatorTestLowCurrentThresh;
			final double kLowRpmThres = TestConstants.kElevatorTestLowRPMThresh;

			ArrayList<MotorDiagnostics> mElevatorDiagArr = new ArrayList<>();
			mElevatorDiagArr.add(new MotorDiagnostics("Elevator Motor Master", mElevatorMaster, TestConstants.kElevatorTestSpeed, TestConstants.kElevatorTestDuration, false));
			mElevatorDiagArr.add(new MotorDiagnostics("Elevator Motor Slave 1", mElevatorSlaveA, mElevatorMaster, TestConstants.kElevatorTestSpeed, TestConstants.kElevatorTestDuration, false));
			mElevatorDiagArr.add(new MotorDiagnostics("Elevator Motor Slave 2", mElevatorSlaveB, mElevatorMaster, TestConstants.kElevatorTestSpeed, TestConstants.kElevatorTestDuration, false));
			mElevatorDiagArr.add(new MotorDiagnostics("Elevator Motor Slave 3", mElevatorSlaveC, mElevatorMaster, TestConstants.kElevatorTestSpeed, TestConstants.kElevatorTestDuration, false));

			boolean failure = false;

			for (MotorDiagnostics mD : mElevatorDiagArr) {
				mD.setZero();
			}

			for (MotorDiagnostics mD : mElevatorDiagArr) {
				mD.runTest();

				if (mD.isCurrentUnderThreshold(kLowCurrentThres)) {
					ConsoleReporter.report("!!!!!!!!!!!!!!!!!! " + mD.getMotorName() + " Current Low !!!!!!!!!!");
					failure = true;
				}

				if (mD.isRPMUnderThreshold(kLowRpmThres)) {
					ConsoleReporter.report("!!!!!!!!!!!!!!!!!! " + mD.getMotorName() + " RPM Low !!!!!!!!!!");
					failure = true;
				}

				if (!mD.isSensorInPhase()) {
					ConsoleReporter.report("!!!!!!!!!!!!!!!!!! " + mD.getMotorName() + " Sensor Out of Phase !!!!!!!!!!");
					failure = true;
				}

			}

			if (mElevatorDiagArr.size() > 0) {
				List<Double> elevatorMotorCurrents = mElevatorDiagArr.stream().map(MotorDiagnostics::getMotorCurrent).collect(Collectors.toList());
				if (!Util.allCloseTo(elevatorMotorCurrents, elevatorMotorCurrents.get(0), TestConstants.kElevatorTestCurrentDelta)) {
					failure = true;
					ConsoleReporter.report("!!!!!!!!!!!!!!!!!! Elevator Motor Currents Different !!!!!!!!!!");
				}

				List<Double> elevatorMotorRPMs = mElevatorDiagArr.stream().map(MotorDiagnostics::getMotorRPM).collect(Collectors.toList());
				if (!Util.allCloseTo(elevatorMotorRPMs, elevatorMotorRPMs.get(0), TestConstants.kElevatorTestRPMDelta)) {
					failure = true;
					ConsoleReporter.report("!!!!!!!!!!!!!!!!!!! Elevator RPMs different !!!!!!!!!!!!!!!!!!!");
				}
			} else {
				ConsoleReporter.report("Elevator Testing Error Occurred in system. Please check code!", MessageLevel.ERROR);
			}

			return !failure;
		}
		else
			return true;
	}

	@Override
	public synchronized String generateReport() {
		return mLogDataGenerator.generateData(mPeriodicIO);

//		return  "ElevatorPos:" + mElevatorMaster.getVelocity() + ";" +
//				"ElevatorVel:" + mElevatorMaster.getVelocity() + ";" +
//				"ElevatorOutput:" + mPeriodicIO.elevator_setpoint + ";" +
//				"Elevator1Current:" + mElevatorMaster.getMCOutputCurrent() + ";" +
//				"Elevator2Current:" + mElevatorSlaveA.getMCOutputCurrent() + ";" +
//				"Elevator3Current:" + mElevatorSlaveB.getMCOutputCurrent() + ";" +
//				"Elevator4Current:" + mElevatorSlaveC.getMCOutputCurrent() + ";" +
//				"ElevatorOutputDutyCycle:" + mElevatorMaster.getMCOutputPercent() + ";" +
//				"ElevatorOutputVoltage:" + mElevatorMaster.getMCOutputPercent() * mElevatorMaster.getMCInputVoltage() + ";" +
//				"ElevatorSupplyVoltage:" + mElevatorMaster.getMCInputVoltage() + ";" +
//				"Elevator1HasReset:" + mElevatorMaster.hasMotorControllerReset().getMessage() + ";" +
//				"Elevator2HasReset:" + mElevatorSlaveA.hasMotorControllerReset().getMessage() + ";" +
//				"Elevator3HasReset:" + mElevatorSlaveB.hasMotorControllerReset().getMessage() + ";" +
//				"Elevator4HasReset:" + mElevatorSlaveC.hasMotorControllerReset().getMessage() + ";" +
//				"ElevatorControlMode:" + mElevatorControlMode.toString() + ";" +
//				"IsElevatorFaulted:" + isSystemFaulted() + ";";
	}

	@Override
	public void zeroSensors() {
		mElevatorMaster.setEncoderPosition(0);
//		mPeriodicIO = new PeriodicIO();
	}

	public void zeroDriveEncoders() {
		mElevatorSlaveA.setEncoderPosition(0);
		mElevatorSlaveB.setEncoderPosition(0);
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
						double outputPos = mPeriodicIO.elevator_setpoint;

						boolean eUp = requestMoveElevatorUpCheck.hasPassedConditions() && requestMoveElevatorUpCheck.isEnabled();
						boolean eDown = requestMoveElevatorDownCheck.hasPassedConditions() && requestMoveElevatorDownCheck.isEnabled();

						if (eUp) {
							outputPos = Math.max(outputPos, ElevatorPositions.CollisionThresholdHatchArm + ElevatorPositions.PositionDelta);
						}
						if (eDown && getPosition() <= ElevatorPositions.CargoBall) {
							outputPos = Math.min(outputPos, ElevatorPositions.CollisionThresholdBallArm - ElevatorPositions.PositionDelta);
						}

						if (eUp || eDown || (!eUp && !eDown))
							mElevatorMaster.set(MCControlMode.MotionMagic, outputPos, 0, 0);

						break;
					case OPEN_LOOP:
						mElevatorMaster.set(MCControlMode.PercentOut, Math.min(Math.max(mPeriodicIO.elevator_setpoint, -1), 1), 0, 0);
						break;
					case DISABLED:
						mElevatorMaster.set(MCControlMode.Disabled, 0, 0, 0);
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
			return "Elevator";
		}
	};

	public void setCollisionAvoidanceEnabled(boolean enabled) {
		requestMoveElevatorDownCheck.setEnabled(enabled);
		requestMoveElevatorUpCheck.setEnabled(enabled);
	}

	public boolean isLeftDriveEncoderPresent() {
		return mElevatorSlaveA.isEncoderPresent();
	}

	public boolean isRightDriveEncoderPresent() {
		return mElevatorSlaveB.isEncoderPresent();
	}

	public double getLeftDrivePosition() {
		return mElevatorSlaveA.getPosition();
	}

	public double getRightDrivePosition() {
		return mElevatorSlaveB.getPosition();
	}

	public double getLeftDriveVelocity() {
		return mElevatorSlaveA.getVelocity();
	}

	public double getRightDriveVelocity() {
		return mElevatorSlaveB.getVelocity();
	}

	public boolean isAtLowerLimit() {
		return mPeriodicIO.elevator_at_lower_limit;
	}

	@Override
	public double getPosition() {
		return mPeriodicIO.elevator_position;
	}

	@Override
	public double getSetpoint() {
		return mPeriodicIO.elevator_setpoint;
	}

	public synchronized void setElevatorPosition(double elevatorPosition) {
		mPeriodicIO.elevator_setpoint = elevatorPosition;
	}

	public boolean isElevatorAtSetpoint(double posDelta) {
		return Math.abs(mPeriodicIO.elevator_setpoint - mPeriodicIO.elevator_position) < Math.abs(posDelta);
	}

	public synchronized void setElevatorControlMode (ElevatorControlMode elevatorControlMode) {
		mElevatorControlMode = elevatorControlMode;
	}

	public enum ElevatorControlMode {
		POSITION,
		OPEN_LOOP,
		DISABLED;
	}

	@Override
	public synchronized void readPeriodicInputs() {
		mPeriodicIO.elevator_position = mElevatorMaster.getPosition();
		mPeriodicIO.elevator_at_lower_limit = mElevatorSlaveC.getReverseLimitValue();
		mPeriodicIO.elevator_master_reset = mElevatorMasterHasReset.getValue();
		mPeriodicIO.elevator_encoder_present = mElevatorEncoderPresent.getValue();
	}

	@Override
	public synchronized void writePeriodicOutputs() {

	}

	public static class PeriodicIO {
		//Making members public here will automatically add them to logs
		// INPUTS
		public double elevator_position;
		public double elevator_setpoint;
		public boolean elevator_at_lower_limit;
		public boolean elevator_master_reset;
		public boolean elevator_encoder_present;
	}
}
