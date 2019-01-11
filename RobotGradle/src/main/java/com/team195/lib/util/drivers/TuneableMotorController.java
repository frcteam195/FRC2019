package com.team195.lib.util.drivers;

public interface TuneableMotorController {
	void setPIDF(double kP, double kI, double kD, double kF);
	void setIZone(double iZone);
	void setIAccum(double iAccum);
	void setMaxIAccum(double maxIAccum);
	void setRampRate(double rampRate);
	void setMotionParameters(int cruiseVel, int cruiseAccel);

	/**
	 * Method to change control modes. Make sure this method only changes modes if the current mode is not the desired mode.
	 * @param controlMode The desired control mode
	 */
	void setControlMode(MCControlMode controlMode);

	////////////
	//Units expected in standard form, i.e. Rotations, RPM, etc.
	void setSetpoint(double setpoint);
	double getActual();
	////////////

	double getIntegralAccum();

	MCControlMode getMotionControlMode();

	public enum MCControlMode {
		PERCENTOUT,
		POSITION,
		VELOCITY,
		CURRENT,
		VOLTAGE,
		MOTIONMAGIC,
		INVALID
	}
}
