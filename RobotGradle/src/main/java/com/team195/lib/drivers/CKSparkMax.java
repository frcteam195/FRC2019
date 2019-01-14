package com.team195.lib.drivers;

import com.revrobotics.*;
import java.util.Optional;

public class CKSparkMax extends CANSparkMax implements TuneableMotorController {
	private int currentSelectedSlot = 0;
	private final CANPIDController canPIDController;
	private final CANEncoder canEncoder;
	private double prevOutput = Double.MIN_VALUE;

	private Configuration fastMasterConfig = new Configuration(5, 5, 20, 50);
	private Configuration normalMasterConfig = new Configuration(10, 10, 20, 50);
	private Configuration normalSlaveConfig = new Configuration(10, 100, 100, 100);

	private double voltageCompensation = 12;

	public CKSparkMax(int deviceID, MotorType type, boolean fastMaster) {
		super(deviceID, type);
		Configuration config = fastMaster ? fastMasterConfig : normalMasterConfig;
		canPIDController = getPIDController();
		canEncoder = getEncoder();
		canPIDController.setOutputRange(-1, 1);
		setControlFramePeriod(config.CONTROL_FRAME_PERIOD_MS);
		setPeriodicFramePeriod(PeriodicFrame.kStatus0, config.STATUS_FRAME_0_MS);
		setPeriodicFramePeriod(PeriodicFrame.kStatus1, config.STATUS_FRAME_1_MS);
		setPeriodicFramePeriod(PeriodicFrame.kStatus2, config.STATUS_FRAME_2_MS);
		setSmartCurrentLimit(80);
	}

	public CKSparkMax(int deviceID, MotorType type, CANSparkMax masterSpark) {
		super(deviceID, type);
		follow(masterSpark);
		Configuration config = normalSlaveConfig;
		canPIDController = getPIDController();
		canEncoder = getEncoder();
		setControlFramePeriod(config.CONTROL_FRAME_PERIOD_MS);
		setPeriodicFramePeriod(PeriodicFrame.kStatus0, config.STATUS_FRAME_0_MS);
		setPeriodicFramePeriod(PeriodicFrame.kStatus1, config.STATUS_FRAME_1_MS);
		setPeriodicFramePeriod(PeriodicFrame.kStatus2, config.STATUS_FRAME_2_MS);
		setSmartCurrentLimit(80);
	}

	@Deprecated
	public void set(double DO_NOT_USE) {

	}

	@Override
	public void set(MCControlMode controlMode, double demand, int slotIdx, double arbitraryFeedForward) {
		if (currentSelectedSlot != slotIdx)
			setPIDGainSlot(slotIdx);

		//TODO: Remove if not necessary after testing position units
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

		if (demand + arbitraryFeedForward != prevOutput || currentSelectedSlot != slotIdx || controlMode.Rev() != getControlType()) {
			canPIDController.setReference(demand, controlMode.Rev(), slotIdx, arbitraryFeedForward * voltageCompensation);
			prevOutput = demand + arbitraryFeedForward;
		}
	}

	public double getPosition() {
		return canEncoder.getPosition();
	}

	public double getVelocity() {
		return canEncoder.getVelocity();
	}

	@Override
	public double getSensorUnitsPerRotation() {
//		return 42;
		//TODO: Change this after testing what position units are in for Rev
		return 1;
	}

	@Override
	public double getVelocityRPMTimeConversionFactor() {
		return 1;
	}


	/**
	 * Config voltage compensation for arbitrary feed forward
	 * @param voltage Nominal voltage setting for arbitrary feed forward compensation
	 */
	public synchronized void configVoltageCompSaturation(double voltage) {
		voltageCompensation = voltage;
	}

	@Override
	public void setPIDF(double kP, double kI, double kD, double kF) {
		canPIDController.setP(kP, currentSelectedSlot);
		canPIDController.setI(kI, currentSelectedSlot);
		canPIDController.setD(kD, currentSelectedSlot);
		canPIDController.setFF(kF, currentSelectedSlot);
	}

	@Override
	public void setIZone(double iZone) {
		canPIDController.setIZone(iZone, currentSelectedSlot);
	}

	@Override
	public void setIAccum(double iAccum) {

	}

	@Override
	public void setMaxIAccum(double maxIAccum) {

	}

	@Override
	public void setMCRampRate(double rampRate) {
		setRampRate(rampRate);
	}

	@Override
	public void setMotionParameters(int cruiseVel, int cruiseAccel) {

	}

	@Override
	public void setPIDGainSlot(int slotIdx) {
		setCurrentSelectedSlot(slotIdx);
	}

	private synchronized void setCurrentSelectedSlot(int slotIdx) {
		currentSelectedSlot = slotIdx;
	}

	@Override
	public void setControlMode(MCControlMode controlMode) {
		if (getMotionControlMode() != controlMode)
		{
			switch (controlMode) {
				case Position:
					set(controlMode, canEncoder.getPosition(), currentSelectedSlot, 0);
					break;
				case PercentOut:
				case Velocity:
				case Voltage:
				default:
					set(controlMode, 0, currentSelectedSlot, 0);
					break;
			}
		}
	}

	@Override
	public void setSetpoint(double setpoint) {
		set(getMotionControlMode(), setpoint, currentSelectedSlot, 0);
	}

	@Override
	public double getActual() {
		switch (getControlType()) {
			case kDutyCycle:
				return getAppliedOutput();
			case kVelocity:
				return canEncoder.getVelocity();
			case kVoltage:
				return getAppliedOutput() * voltageCompensation;
			case kPosition:
				return canEncoder.getPosition();
			default:
				return 0;
		}
	}

	@Override
	public double getIntegralAccum() {
		return 0;
	}

	@Override
	public MCControlMode getMotionControlMode() {
		return MCControlMode.valueOf(getControlType());
	}

	public ControlType getControlType() {
		Optional<Integer> o = getParameterInt(ConfigParameter.kCtrlType);
		if (!o.isPresent())
			return ControlType.kDutyCycle;
		return controlTypeFromInt(o.get());
	}

	private ControlType controlTypeFromInt(int ctrlMode) {
		switch (ctrlMode) {
			case 0:
				return ControlType.kDutyCycle;
			case 1:
				return ControlType.kVelocity;
			case 2:
				return ControlType.kVoltage;
			case 3:
				return ControlType.kPosition;
			default:
				return ControlType.kDutyCycle;
		}
	}

	private static class Configuration {
		int CONTROL_FRAME_PERIOD_MS;
		int STATUS_FRAME_0_MS;
		int STATUS_FRAME_1_MS;
		int STATUS_FRAME_2_MS;

		Configuration(int CONTROL_FRAME_PERIOD_MS, int STATUS_FRAME_0_MS, int STATUS_FRAME_1_MS, int STATUS_FRAME_2_MS) {
			this.CONTROL_FRAME_PERIOD_MS = CONTROL_FRAME_PERIOD_MS;
			this.STATUS_FRAME_0_MS = STATUS_FRAME_0_MS;
			this.STATUS_FRAME_1_MS = STATUS_FRAME_1_MS;
			this.STATUS_FRAME_2_MS = STATUS_FRAME_2_MS;
		}
	}
}
