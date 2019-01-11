package com.team195.lib.util;

import com.team195.frc2019.Constants;

public class QuickMaths {


	public static double normalizeJoystickWithDeadband(double val, double deadband) {
		val = (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;

		if (val != 0)
			val = Math.signum(val) * ((Math.abs(val) - deadband) / (1.0 - deadband));

		return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
	}

	public static double convertAngleToSRX(double angle, double countsPerRev) {
		return angle * countsPerRev / 360;
	}

	public static double convertNativeUnitsToRotations(int nativeUnitsPos) {
		return nativeUnitsPos / Constants.kSensorUnitsPerRotation;
	}

	public static int convertRotationsToNativeUnits(double rotations) {
		return (int)(rotations * Constants.kSensorUnitsPerRotation);
	}

	public static double convertNativeUnitsToRPM(int nativeUnits) {
		return (nativeUnits / Constants.kSensorUnitsPerRotation * Constants.k100msPerMinute);
	}

	public static int convertRPMToNativeUnits(double rpm) {
		return (int)(rpm * Constants.kSensorUnitsPerRotation / Constants.k100msPerMinute);
	}
}
