package com.team195.lib.drivers;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team195.frc2019.Constants;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.reporters.MessageLevel;
import com.team195.lib.util.QuickMaths;

public class CKTalonSRX extends TalonSRX implements TuneableMotorController {
	private int currentSelectedSlot = 0;
	private double[] mCLRampRate = new double[4];
	private int[] mMMAccel = new int[4];
	private int[] mMMVel = new int[4];
	private double prevOutput = Double.MIN_VALUE;

	private final Configuration fastMasterConfig = new Configuration(5, 5, 20);
	private final Configuration normalMasterConfig = new Configuration(10, 10, 20);
	private final Configuration normalSlaveConfig = new Configuration(10, 100, 100);

	public CKTalonSRX(int deviceId, boolean fastMaster) {
		super(deviceId);
		Configuration config = fastMaster ? fastMasterConfig : normalMasterConfig;

		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = true;
			setSucceeded &= clearStickyFaults(Constants.kLongCANTimeoutMs) == ErrorCode.OK;
			setSucceeded &= setControlFramePeriod(ControlFrame.Control_3_General, config.CONTROL_FRAME_PERIOD_MS) == ErrorCode.OK;
			setSucceeded &= setStatusFramePeriod(StatusFrame.Status_1_General, config.STATUS_FRAME_GENERAL_1_MS, Constants.kLongCANTimeoutMs) == ErrorCode.OK;
			setSucceeded &= setStatusFramePeriod(StatusFrame.Status_2_Feedback0, config.STATUS_FRAME_FEEDBACK0_2_MS, Constants.kLongCANTimeoutMs) == ErrorCode.OK;
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to initialize Talon " + getDeviceID() + " !!!!!!", MessageLevel.DEFCON1);
	}

	public CKTalonSRX(int deviceId, TalonSRX masterTalon) {
		super(deviceId);
		follow(masterTalon);
		Configuration config = normalSlaveConfig;

		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = true;
			setSucceeded &= clearStickyFaults(Constants.kLongCANTimeoutMs) == ErrorCode.OK;
			setSucceeded &= setControlFramePeriod(ControlFrame.Control_3_General, config.CONTROL_FRAME_PERIOD_MS) == ErrorCode.OK;
			setSucceeded &= setStatusFramePeriod(StatusFrame.Status_1_General, config.STATUS_FRAME_GENERAL_1_MS, Constants.kLongCANTimeoutMs) == ErrorCode.OK;
			setSucceeded &= setStatusFramePeriod(StatusFrame.Status_2_Feedback0, config.STATUS_FRAME_FEEDBACK0_2_MS, Constants.kLongCANTimeoutMs) == ErrorCode.OK;
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to initialize Talon " + getDeviceID() + " !!!!!!", MessageLevel.DEFCON1);
	}

	@Override
	public ErrorCode configClosedloopRamp(double secondsFromNeutralToFull, int timeoutMs) {
		return configClosedloopRamp(secondsFromNeutralToFull, currentSelectedSlot, timeoutMs);
	}

	public ErrorCode configClosedloopRamp(double secondsFromNeutralToFull, int slotIdx, int timeoutMs) {
		setCurrentSlotCLRampRate(secondsFromNeutralToFull, slotIdx);
		return super.configClosedloopRamp(secondsFromNeutralToFull, timeoutMs);
	}

	@Override
	public ErrorCode configMotionAcceleration(int sensorUnitsPer100msPerSec, int timeoutMs) {
		return configMotionAcceleration(sensorUnitsPer100msPerSec, currentSelectedSlot, timeoutMs);
	}

	public ErrorCode configMotionAcceleration(int sensorUnitsPer100msPerSec, int slotIdx, int timeoutMs) {
		setCurrentMMAccel(sensorUnitsPer100msPerSec, slotIdx);
		return super.configMotionAcceleration(sensorUnitsPer100msPerSec, timeoutMs);
	}

	@Override
	public ErrorCode configMotionCruiseVelocity(int sensorUnitsPer100ms, int timeoutMs) {
		return configMotionCruiseVelocity(sensorUnitsPer100ms, currentSelectedSlot, timeoutMs);
	}

	public ErrorCode configMotionCruiseVelocity(int sensorUnitsPer100ms, int slotIdx, int timeoutMs) {
		setCurrentMMVel(sensorUnitsPer100ms, slotIdx);
		return super.configMotionCruiseVelocity(sensorUnitsPer100ms, timeoutMs);
	}

	/**
	 * Make sure you call this method before first use of set() to ensure CL ramp rate and PID gains are selected properly when using CKTalonSRX
	 * @param slotIdx Gain profile slot
	 * @param pidIdx PID ID, 0 for main, 1 for aux
	 */
	@Override
	public void selectProfileSlot(int slotIdx, int pidIdx) {
		super.selectProfileSlot(slotIdx, pidIdx);
		setCurrentSlotValue(slotIdx);
		if (currentSelectedSlot < mCLRampRate.length && currentSelectedSlot < mMMAccel.length && currentSelectedSlot < mMMVel.length) {
			boolean setSucceeded;
			int retryCounter = 0;

			do {
				setSucceeded = configClosedloopRamp(mCLRampRate[currentSelectedSlot], currentSelectedSlot, Constants.kCANTimeoutMs) == ErrorCode.OK;
				setSucceeded &= configMotionAcceleration(mMMAccel[currentSelectedSlot], currentSelectedSlot, Constants.kCANTimeoutMs) == ErrorCode.OK;
				setSucceeded &= configMotionCruiseVelocity(mMMVel[currentSelectedSlot], currentSelectedSlot, Constants.kCANTimeoutMs) == ErrorCode.OK;
			} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

			if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
				ConsoleReporter.report("Failed to change Talon ID" + getDeviceID() +  " profile slot!!!", MessageLevel.DEFCON1);
		}
	}

	private synchronized void setCurrentSlotValue(int slotIdx) {
		currentSelectedSlot = slotIdx;
	}

	private synchronized void setCurrentSlotCLRampRate(double rampRate, int slot) {
		if (slot < mCLRampRate.length) {
			mCLRampRate[slot] = rampRate;
		}
	}

	private synchronized void setCurrentMMVel(int vel, int slot) {
		if (slot < mMMVel.length) {
			mMMVel[slot] = vel;
		}
	}

	private synchronized void setCurrentMMAccel(int accel, int slot) {
		if (slot < mMMAccel.length) {
			mMMAccel[slot] = accel;
		}
	}

	@Override
	public String toString() {
		StringBuilder sb = new StringBuilder();
		sb.append("General Status Frame 1: " + getStatusFramePeriod(StatusFrameEnhanced.Status_1_General, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 2: " + getStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 3: " + getStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 4: " + getStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 6: " + getStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 7: " + getStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 8: " + getStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 9: " + getStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 10: " + getStatusFramePeriod(StatusFrame.Status_10_Targets, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 11: " + getStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 12: " + getStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 13: " + getStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 14: " + getStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 15: " + getStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmareApiStatus, Constants.kLongCANTimeoutMs) + "\r\n");
		return sb.toString();
	}

	@Deprecated
	public void set(ControlMode mode, double outputValue) {
		set(MCControlMode.valueOf(mode), outputValue, currentSelectedSlot, 0);
	}

	@Deprecated
	public void set(ControlMode mode, double demand0, double demand1) {
		set(MCControlMode.valueOf(mode), demand0, currentSelectedSlot, demand1);
	}

	@Deprecated
	public void set(ControlMode mode, double demand0, DemandType demand1Type, double demand1){
		set(MCControlMode.valueOf(mode), demand0, currentSelectedSlot, demand1);
	}

	@Override
	public void set(MCControlMode controlMode, double demand, int slotIdx, double arbitraryFeedForward) {
		if (currentSelectedSlot != slotIdx)
			selectProfileSlot(slotIdx, 0);

		switch (controlMode) {
			case Position:
			case MotionMagic:
				demand = convertRotationsToNativeUnits(demand);
				break;
			case Velocity:
				demand = convertRPMToNativeUnits(demand);
				break;
			default:
				break;
		}

		if (demand + arbitraryFeedForward != prevOutput || currentSelectedSlot != slotIdx || controlMode.CTRE() != getControlMode()) {
			set(controlMode.CTRE(), demand, DemandType.ArbitraryFeedForward, arbitraryFeedForward);
			prevOutput = demand + arbitraryFeedForward;
		}
	}

	@Override
	public void setPIDF(double kP, double kI, double kD, double kF) {
		config_kP(currentSelectedSlot, kP);
		config_kI(currentSelectedSlot, kI);
		config_kD(currentSelectedSlot, kD);
		config_kF(currentSelectedSlot, kF);
	}

	@Override
	public void setIZone(double iZone) {
		config_IntegralZone(currentSelectedSlot, (int)iZone);
	}

	@Override
	public void setIAccum(double iAccum) {
		setIntegralAccumulator(iAccum);
	}

	@Override
	public void setMaxIAccum(double maxIAccum) {
		configMaxIntegralAccumulator(currentSelectedSlot, maxIAccum);
	}

	@Override
	public void setMCRampRate(double rampRate) {
		configClosedloopRamp(rampRate, Constants.kCANTimeoutMs);
	}

	@Override
	public void setMotionParameters(int cruiseVel, int cruiseAccel) {
		configMotionCruiseVelocity(convertRPMToNativeUnits(cruiseVel), Constants.kCANTimeoutMs);
		configMotionAcceleration(convertRPMToNativeUnits(cruiseAccel), Constants.kCANTimeoutMs);
	}

	@Override
	public void setPIDGainSlot(int slotIdx) {
		if (currentSelectedSlot != slotIdx)
			selectProfileSlot(slotIdx, 0);
	}

	@Override
	public void setControlMode(MCControlMode controlMode) {
		if (getMotionControlMode() != controlMode) {
			switch (controlMode) {
				case PercentOut:
				case Velocity:
				case Current:
					set(controlMode, 0, currentSelectedSlot, 0);
					break;
				case Position:
				case MotionMagic:
					set(controlMode, getPosition(), currentSelectedSlot, 0);
					break;
				default:
					break;
			}
		}
	}

	@Override
	public MCControlMode getMotionControlMode() {
		return MCControlMode.valueOf(getControlMode());
	}

	@Override
	public void setSetpoint(double setpoint) {
		set(getMotionControlMode(), setpoint, currentSelectedSlot, 0);
	}

	@Override
	public double getActual() {
		switch (getControlMode()) {
			case PercentOutput:
				return getMotorOutputPercent();
			case Position:
			case MotionMagic:
				return getPosition();
			case Velocity:
				return getVelocity();
			case Current:
				return getOutputCurrent();
			default:
				return 0;
		}
	}

	@Override
	public double getPosition() {
		return convertNativeUnitsToRotations(getSelectedSensorPosition());
	}

	@Override
	public double getVelocity() {
		return convertNativeUnitsToRPM(getSelectedSensorVelocity());
	}

	@Override
	public double getSensorUnitsPerRotation() {
		return 4096;
	}

	@Override
	public double getVelocityRPMTimeConversionFactor() {
		return 600;
	}

	@Override
	public double getIntegralAccum() {
		return getIntegralAccumulator();
	}

	private static class Configuration {
		int CONTROL_FRAME_PERIOD_MS;
		int STATUS_FRAME_GENERAL_1_MS;
		int STATUS_FRAME_FEEDBACK0_2_MS;

		Configuration(int CONTROL_FRAME_PERIOD_MS, int STATUS_FRAME_GENERAL_1_MS, int STATUS_FRAME_FEEDBACK0_2_MS) {
			this.CONTROL_FRAME_PERIOD_MS = CONTROL_FRAME_PERIOD_MS;
			this.STATUS_FRAME_GENERAL_1_MS = STATUS_FRAME_GENERAL_1_MS;
			this.STATUS_FRAME_FEEDBACK0_2_MS = STATUS_FRAME_FEEDBACK0_2_MS;
		}
	}
}
