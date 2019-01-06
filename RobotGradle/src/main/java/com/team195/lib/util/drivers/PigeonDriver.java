package com.team195.lib.util.drivers;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.team195.frc2019.Constants;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.reporters.MessageLevel;

public class PigeonDriver {
	private PigeonIMU pigeonIMU;

	public PigeonDriver(int deviceNumber) {
		pigeonIMU = new PigeonIMU(deviceNumber);
	}

	public boolean reset() {
		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = pigeonIMU.setYaw(0, Constants.kCANTimeoutMs) == ErrorCode.OK;
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to reset Pigeon " + pigeonIMU.getDeviceID() + " !!!!!!", MessageLevel.DEFCON1);

		return setSucceeded;
	}
}
