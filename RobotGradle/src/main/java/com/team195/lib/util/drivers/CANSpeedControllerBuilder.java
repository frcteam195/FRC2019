package com.team195.lib.util.drivers;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team195.frc2019.Constants;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.reporters.MessageLevel;

public class CANSpeedControllerBuilder {
	private static class Configuration {
		public double MAX_OUTPUT = 1;
		public double NOMINAL_OUTPUT = 0;
		public NeutralMode NEUTRAL_MODE = NeutralMode.Brake;
		public boolean ENABLE_CURRENT_LIMIT = false;
		public boolean ENABLE_SOFT_LIMIT = false;
		//public boolean ENABLE_LIMIT_SWITCH = false;
		public int CURRENT_LIMIT = 0;
		public boolean INVERTED = false;

		public int CONTROL_FRAME_PERIOD_MS = 10;
		public int STATUS_FRAME_GENERAL_1_MS = 10;
		public int STATUS_FRAME_FEEDBACK0_2_MS = 20;
		public int STATUS_FRAME_QUADRATURE_3_MS = 160;
		public int STATUS_FRAME_ANALOG_4_MS = 160;
		public int STATUS_FRAME_PULSE_8_MS = 160;
		public int STATUS_FRAME_TARGET_10_MS = 0;
		public int STATUS_FRAME_UART_11_MS = 250;
		public int STATUS_FRAME_FEEDBACK1_12_MS = 250;
		public int STATUS_FRAME_PIDF0_13_MS = 160;
		public int STATUS_FRAME_PIDF1_14_MS = 250;
		public int STATUS_FRAME_FIRMWARE_15_MS = 160;

		//public VelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD = VelocityMeasPeriod::Period_100Ms;
		//public int VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 64;
	}
	
	private static Configuration kDefaultConfiguration = new Configuration();
	private static Configuration kSlaveConfiguration = new Configuration();

	static {
		kSlaveConfiguration.STATUS_FRAME_GENERAL_1_MS = 100;
		kSlaveConfiguration.STATUS_FRAME_FEEDBACK0_2_MS = 100;
	}
	
	private CANSpeedControllerBuilder() { }

	public static CKTalonSRX createDefaultTalonSRX(int id) {
		return createTalonSRX(id, kDefaultConfiguration);
	}

	public static CKTalonSRX createMasterTalonSRX(int id) {
		Configuration masterConfig = new Configuration();
		return createTalonSRX(id, masterConfig);
	}

	public static CKTalonSRX createFastMasterTalonSRX(int id) {
		Configuration masterConfig = new Configuration();
		masterConfig.CONTROL_FRAME_PERIOD_MS = 5;
		masterConfig.STATUS_FRAME_GENERAL_1_MS = 5;
		return createTalonSRX(id, masterConfig);
	}
	
	public static CKTalonSRX createPermanentSlaveTalonSRX(int id, TalonSRX masterTalon) {
		CKTalonSRX talon = createTalonSRX(id, kSlaveConfiguration);
		talon.follow(masterTalon);
		return talon;
	}

	public static CKTalonSRX createTalonSRX(int id, Configuration config) {
        CKTalonSRX talon = new CKTalonSRX(id);
        configTalon(talon, config);
        return talon;
    }

	private static boolean configTalon(TalonSRX talon, Configuration config) {
		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = true;
			setSucceeded &= talon.clearStickyFaults(Constants.kLongCANTimeoutMs) == ErrorCode.OK;
			setSucceeded &= talon.setControlFramePeriod(ControlFrame.Control_3_General, config.CONTROL_FRAME_PERIOD_MS) == ErrorCode.OK;
			setSucceeded &= talon.setStatusFramePeriod(StatusFrame.Status_1_General, config.STATUS_FRAME_GENERAL_1_MS, Constants.kLongCANTimeoutMs) == ErrorCode.OK;
			setSucceeded &= talon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, config.STATUS_FRAME_FEEDBACK0_2_MS, Constants.kLongCANTimeoutMs) == ErrorCode.OK;
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to initialize Talon " + talon.getDeviceID() + " !!!!!!", MessageLevel.DEFCON1);

		return setSucceeded;
	}

}