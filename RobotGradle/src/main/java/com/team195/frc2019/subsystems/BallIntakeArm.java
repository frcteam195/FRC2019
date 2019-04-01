package com.team195.frc2019.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.team195.frc2019.auto.actions.SetBallArmRotationAction;
import com.team195.frc2019.auto.autonomy.AutomatedAction;
import com.team195.frc2019.constants.CalConstants;
import com.team195.frc2019.auto.autonomy.AutomatedActions;
import com.team195.frc2019.constants.Constants;
import com.team195.frc2019.constants.DeviceIDConstants;
import com.team195.frc2019.constants.TestConstants;
import com.team195.frc2019.loops.ILooper;
import com.team195.frc2019.loops.Loop;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.reporters.DiagnosticMessage;
import com.team195.frc2019.reporters.MessageLevel;
import com.team195.frc2019.subsystems.positions.BallIntakeArmPositions;
import com.team195.frc2019.subsystems.positions.ElevatorPositions;
import com.team195.frc2019.subsystems.positions.TurretPositions;
import com.team195.lib.drivers.CKSolenoid;
import com.team195.lib.drivers.motorcontrol.CKTalonSRX;
import com.team195.lib.drivers.motorcontrol.MCControlMode;
import com.team195.lib.drivers.motorcontrol.PDPBreaker;
import com.team195.lib.util.*;

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
		mBallArmRotationMotor = new CKTalonSRX(DeviceIDConstants.kBallIntakeRotationMotorId, false, PDPBreaker.B30A);
		mBallArmRotationMotor.setSensorPhase(true);

		mBallArmRollerMotor = new CKTalonSRX(DeviceIDConstants.kBallIntakeRollerMotorId, false, PDPBreaker.B30A);
		mBallArmRollerMotor.setInverted(true);
		mBallArmRollerMotor.setSensorPhase(true);
		mBallArmRollerMotor.setMCOpenLoopRampRate(0.2);
		mBallArmRollerMotor.configCurrentLimit(20, 25, 100);

		mBallArmRotationMotor.setPIDGainSlot(0);
		mBallArmRotationMotor.setFeedbackDevice(RemoteFeedbackDevice.RemoteSensor0, DeviceIDConstants.kBallIntakeRollerMotorId);
		mBallArmRotationMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
		mBallArmRotationMotor.setPIDF(CalConstants.kBallIntakeArmUpPositionKp, CalConstants.kBallIntakeArmUpPositionKi, CalConstants.kBallIntakeArmUpPositionKd, CalConstants.kBallIntakeArmUpPositionKf);
		mBallArmRotationMotor.setMotionParameters(CalConstants.kBallIntakeArmUpPositionCruiseVel, CalConstants.kBallIntakeArmUpPositionMMAccel);
		mBallArmRotationMotor.setPIDGainSlot(1);
		mBallArmRotationMotor.setFeedbackDevice(RemoteFeedbackDevice.RemoteSensor0, DeviceIDConstants.kBallIntakeRollerMotorId);
		mBallArmRotationMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
		mBallArmRotationMotor.setPIDF(CalConstants.kBallIntakeArmDownPositionKp, CalConstants.kBallIntakeArmDownPositionKi, CalConstants.kBallIntakeArmDownPositionKd, CalConstants.kBallIntakeArmDownPositionKf);
		mBallArmRotationMotor.setMotionParameters(CalConstants.kBallIntakeArmDownPositionCruiseVel, CalConstants.kBallIntakeArmDownPositionMMAccel);

		mBallArmRotationMotor.setPIDGainSlot(0);
		trc.start();
		trc.doRateControl(100);
		zeroSensors();
		trc.doRateControl(100);
//		mBallArmRotationMotor.configForwardSoftLimitThreshold(CalConstants.kBallIntakeArmForwardSoftLimit);
		mBallArmRotationMotor.configForwardSoftLimitEnable(false);
		mBallArmRotationMotor.configReverseSoftLimitEnable(false);
		mBallArmRotationMotor.configCurrentLimit(10, 12, 200);
		mBallArmRotationMotor.setControlMode(MCControlMode.Disabled);

//		TuneablePIDOSC x;
//		try {
//			x = new TuneablePIDOSC("Ball Arm", 5804, true, mBallArmRotationMotor);
//		} catch (Exception ignored) {
//
//		}

		ballArmUpCheck = new MotionInterferenceChecker(MotionInterferenceChecker.LogicOperation.AND, true,
				(t) -> (Elevator.getInstance().getPosition() < ElevatorPositions.CollisionThresholdBallArm) || (Elevator.getInstance().getPosition() > ElevatorPositions.CargoBall),
				(t) -> ((Math.abs(Turret.getInstance().getPosition() - TurretPositions.Home) < TurretPositions.PositionDelta)
						|| (Math.abs(Turret.getInstance().getPosition() - TurretPositions.Back180) < TurretPositions.PositionDelta))
		);

		mBallIntakeBarDropSolenoid = new CKSolenoid(DeviceIDConstants.kBallIntakeBarSolenoidId);
	}

	public static BallIntakeArm getInstance() {
		return mInstance;
	}

	@Override
	public void stop() {

	}

	@Override
	public synchronized boolean isSystemFaulted() {
		boolean systemFaulted = !mBallArmRollerMotor.isEncoderPresent();

		if (systemFaulted) {
			ConsoleReporter.report("Ball Intake Arm Encoder Missing!", MessageLevel.DEFCON1);
		}

		systemFaulted |= mBallArmRollerMotor.hasMotorControllerReset() != DiagnosticMessage.NO_MSG;

		if (systemFaulted) {
			ConsoleReporter.report("Ball Intake Arm Disabled!", MessageLevel.DEFCON1);
			setBallIntakeArmControlMode(BallIntakeArmControlMode.DISABLED);
		}

		return systemFaulted;
	}

	@Override
	public boolean runDiagnostics() {
		if (TestConstants.ENABLE_BALL_INTAKE_ARM_TEST) {
			ConsoleReporter.report("Testing Ball Intake Arm---------------------------------");
			MotorDiagnostics ballArmRotationDiag;

			boolean failure = false;

			boolean ballArmSensorOk;

			if (isArmUp()) {
				ballArmRotationDiag = new MotorDiagnostics("Ball Arm Rotation Motor", mBallArmRotationMotor,
						TestConstants.kBallArmRotationTestSpeed, TestConstants.kBallArmRotationTestDurationDown, true);
				ballArmSensorOk = true;
			}
			else {
				ballArmRotationDiag = new MotorDiagnostics("Ball Arm Rotation Motor", mBallArmRotationMotor,
						TestConstants.kBallArmRotationTestSpeed, TestConstants.kBallArmRotationTestDurationUp, false);
				ballArmSensorOk = false;
			}

			ballArmRotationDiag.setZero();

			ballArmRotationDiag.runTest();

			ballArmSensorOk |= isArmUp();

			if (ballArmRotationDiag.isCurrentUnderThreshold(TestConstants.kBallArmRotationTestLowCurrentThresh)) {
				ConsoleReporter.report("!!!!!!!!!!!!!!!!!! " + ballArmRotationDiag.getMotorName() + " Current Low !!!!!!!!!!");
				failure = true;
			}

			if (ballArmRotationDiag.isRPMUnderThreshold(TestConstants.kBallArmRotationTestLowRPMThresh)) {
				ConsoleReporter.report("!!!!!!!!!!!!!!!!!! " + ballArmRotationDiag.getMotorName() + " RPM Low !!!!!!!!!!");
				failure = true;
			}

			if (!ballArmSensorOk) {
				ConsoleReporter.report("!!!!!!!!!!!!!!!!!! " + ballArmRotationDiag.getMotorName() + " Limit Switch Missing !!!!!!!!!!");
				failure = true;
			}

			if (!mBallArmRollerMotor.isEncoderPresent()) {
				ConsoleReporter.report("!!!!!!!!!!!!!!!!!! " + ballArmRotationDiag.getMotorName() + " Encoder Missing !!!!!!!!!!");
				failure = true;
			}

			return failure;
		}

		return false;
	}

	@Override
	public synchronized String generateReport() {
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
				"BallArmIntakeSupplyVoltage:" + mBallArmRollerMotor.getMCInputVoltage() + ";" +
				"IsBallIntakeArmFaulted:" + isSystemFaulted() + ";";
	}

	@Override
	public void zeroSensors() {
		mBallArmRotationMotor.setEncoderPosition(CalConstants.kBallIntakeArmForwardSoftLimit);
		zeroRemoteSensor();
	}

	public void zeroRemoteSensor() {
		mBallArmRollerMotor.setEncoderPosition(0);
	}

	public void setSensorsForReset() {
		mBallArmRotationMotor.setLocalQuadPosition(0);
		mBallArmRotationMotor.setEncoderPosition(0);
		zeroRemoteSensor();
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
//
//				if (!isArmUp())
//					TeleopActionRunner.runAction(AutomatedActions.ballArmSet(BallIntakeArmPositions.Up), true);

				if (isArmUp())
					TeleopActionRunner.runAction(AutomatedActions.unfold());
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
						mBallArmRotationMotor.set(MCControlMode.MotionMagic, mBallIntakeArmSetpoint, 0, 0);
						break;
					case OPEN_LOOP:
						if (ballArmUpCheck.hasPassedConditions())
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
//		mBallArmRotationMotor.configCurrentLimit(17, 25, 250);
		mBallArmRotationMotor.configForwardSoftLimitEnable(false);
		mBallArmRotationMotor.configReverseSoftLimitEnable(false);
	}

	public void dropClimbBar() {
		mBallIntakeBarDropSolenoid.set(true);
	}

	@Override
	public double getPosition() {
		return mBallArmRotationMotor.getPosition();
	}

	public double getRemotePosition() {
		return mBallArmRollerMotor.getPosition();
	}

	public boolean isArmUp() {
		return mBallArmRotationMotor.getForwardLimitValue();
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
