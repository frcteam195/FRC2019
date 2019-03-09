package com.team195.lib.util;

import com.team195.frc2019.Constants;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.reporters.MessageLevel;
import com.team195.lib.drivers.motorcontrol.MCControlMode;
import com.team195.lib.drivers.motorcontrol.TuneableMotorController;
import edu.wpi.first.wpilibj.Timer;

public class MotorDiagnostics {
	private static final String TEST_NOT_RUN_STR = "Test not run yet!";

	private double mTestDurationSeconds = 3.0;
	private double mMotorSpeed = 0.75;
	private TuneableMotorController testingSpeedController;
	private TuneableMotorController masterSC = null;
	private String motorName;
	private double motorRPM;
	private double motorCurrent;
	private boolean inverted = false;
	private boolean sensorInPhase = false;
	private boolean testCompleted = false;

	public MotorDiagnostics(String motorName, TuneableMotorController testingSpeedController) {
		this.testingSpeedController = testingSpeedController;
		this.motorName = motorName;
	}

	public MotorDiagnostics(String motorName, TuneableMotorController testingSpeedController, double motorSpeed) {
		this(motorName, testingSpeedController);
		this.mMotorSpeed = motorSpeed;
	}

	public MotorDiagnostics(String motorName, TuneableMotorController testingSpeedController, double motorSpeed, double testDurationSeconds, boolean inverted) {
		this(motorName, testingSpeedController, motorSpeed);
		this.inverted = inverted;
		this.mTestDurationSeconds = testDurationSeconds;
	}

	public MotorDiagnostics(String motorName, TuneableMotorController testingSpeedController, TuneableMotorController masterSC) {
		this(motorName, testingSpeedController);
		this.masterSC = masterSC;
	}

	public MotorDiagnostics(String motorName, TuneableMotorController testingSpeedController, TuneableMotorController masterSC, double motorSpeed) {
		this(motorName, testingSpeedController, motorSpeed);
		this.masterSC = masterSC;
	}

	public MotorDiagnostics(String motorName, TuneableMotorController testingSpeedController, TuneableMotorController masterSC, double motorSpeed, double testDurationSeconds, boolean inverted) {
		this(motorName, testingSpeedController, motorSpeed, testDurationSeconds, inverted);
		this.masterSC = masterSC;
	}

	public void runTest() {
		testingSpeedController.disableSoftLimits();

		double motorPositionPreTest = getPosition();
		mMotorSpeed = Math.abs(mMotorSpeed);
		testingSpeedController.set(MCControlMode.PercentOut, inverted ? -mMotorSpeed : mMotorSpeed, 0, 0);
		Timer.delay(mTestDurationSeconds/2.0);
		motorCurrent = testingSpeedController.getMCOutputCurrent();
		motorRPM = getRPM();
		double motorPositionPostTest = getPosition();
		Timer.delay(mTestDurationSeconds/2.0);
		testingSpeedController.set(MCControlMode.PercentOut, 0, 0, 0);

		if (inverted & (motorPositionPostTest < motorPositionPreTest))
			sensorInPhase =  true;
		else if (!inverted & (motorPositionPostTest > motorPositionPreTest))
			sensorInPhase = true;
		else
			sensorInPhase = false;

		testCompleted = true;
	}

	public boolean isSensorInPhase() {
		if (testCompleted)
			return sensorInPhase;
		else {
			ConsoleReporter.report(TEST_NOT_RUN_STR, MessageLevel.ERROR);
			return false;
		}
	}

	private double getPosition() {
		return masterSC == null ? testingSpeedController.getPosition() : masterSC.getPosition();
	}

	private double getRPM() {
		return masterSC == null ? testingSpeedController.getVelocity() : masterSC.getVelocity();
	}

	public void setZero() {
		testingSpeedController.set(MCControlMode.PercentOut, 0, 0, 0);
	}

	public String getMotorName() {
		return motorName;
	}

	public double getMotorCurrent() {
		if (testCompleted)
			return motorCurrent;
		else {
			ConsoleReporter.report(TEST_NOT_RUN_STR, MessageLevel.ERROR);
			return 0;
		}
	}

	public double getMotorRPM() {
		if (testCompleted)
			return motorRPM;
		else {
			ConsoleReporter.report(TEST_NOT_RUN_STR, MessageLevel.ERROR);
			return 0;
		}
	}

	public boolean isCurrentUnderThreshold(double threshold) {
		if (testCompleted)
			return motorCurrent < threshold;
		else {
			ConsoleReporter.report(TEST_NOT_RUN_STR, MessageLevel.ERROR);
			return true;
		}
	}

	public boolean isRPMUnderThreshold(double threshold) {
		if (testCompleted)
			return motorRPM < threshold;
		else {
			ConsoleReporter.report(TEST_NOT_RUN_STR, MessageLevel.ERROR);
			return true;
		}
	}

	@Override
	public String toString() {
		String retVal = "";
		retVal += motorName + "\r\n";
		retVal += "\tCurrent: " + motorCurrent + "\r\n";
		retVal += "\tRPM: " + motorRPM + "\r\n";
		return retVal;
	}

}
