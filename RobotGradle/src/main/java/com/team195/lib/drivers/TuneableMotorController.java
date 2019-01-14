package com.team195.lib.drivers;

/**
 * Interface for all motor control to abstract controllers from different vendors
 * Units expected in standard form, i.e. Rotations, RPM, etc.
 * All production set commands expected to be differential except those used for tuning
 */
public interface TuneableMotorController {
	void set(MCControlMode controlMode, double demand, int slotIdx, double arbitraryFeedForward);

	void setPIDF(double kP, double kI, double kD, double kF);

	void setIZone(double iZone);

	void setIAccum(double iAccum);

	void setMaxIAccum(double maxIAccum);

	void setMCRampRate(double rampRate);

	void setMotionParameters(int cruiseVel, int cruiseAccel);

	void setPIDGainSlot(int slotIdx);

	/**
	 * Method to change control modes. Make sure this method only changes modes if the current mode is not the desired mode.
	 *
	 * @param controlMode The desired control mode
	 */
	void setControlMode(MCControlMode controlMode);

	void setSetpoint(double setpoint);

	double getActual();

	double getPosition();

	double getVelocity();

	double getSensorUnitsPerRotation();

	double getVelocityRPMTimeConversionFactor();

	double getIntegralAccum();

	MCControlMode getMotionControlMode();

	default double convertNativeUnitsToRotations(double nativeUnitsPos) {
		return nativeUnitsPos / getSensorUnitsPerRotation();
	}

	default int convertRotationsToNativeUnits(double rotations) {
		return (int) (rotations * getSensorUnitsPerRotation());
	}

	default double convertNativeUnitsToRPM(int nativeUnits) {
		return (nativeUnits / getSensorUnitsPerRotation() * getVelocityRPMTimeConversionFactor());
	}

	default int convertRPMToNativeUnits(double rpm) {
		return (int) (rpm * getSensorUnitsPerRotation() / getVelocityRPMTimeConversionFactor());
	}
}
