package com.team195.lib.drivers;

import com.revrobotics.*;
import java.util.Optional;

public class CKSparkMax extends CANSparkMax implements TuneableMotorController {
	private final CANPIDController canPIDController;
	private final CANEncoder canEncoder;

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

	@Override
	public void set(double DO_NOT_USE) {

	}

	public void set(double value, ControlType ctrl) {
		canPIDController.setReference(value, ctrl);
	}

	/**
	 * Four parameter set mode
	 * @param value Output value
	 * @param ctrl Control Mode
	 * @param pidSlot PID Slot
	 * @param arbFeedforward Expected value -1 to 1 and convert it automatically to voltage based on nominal voltage compensation
	 */
	public void set(double value, ControlType ctrl, int pidSlot, double arbFeedforward) {
		canPIDController.setReference(value, ctrl, pidSlot, arbFeedforward * voltageCompensation);
	}

	public double getPosition() {
		return canEncoder.getPosition();
	}

	public double getVelocity() {
		return canEncoder.getVelocity();
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
		canPIDController.setP(kP);
		canPIDController.setI(kI);
		canPIDController.setD(kD);
		canPIDController.setFF(kF);
	}

	@Override
	public void setIZone(double iZone) {
		canPIDController.setIZone(iZone);
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
	public void setControlMode(MCControlMode controlMode) {
		if (getMotionControlMode() != controlMode)
		{
			switch (controlMode) {
				case PercentOut:
					set(0, ControlType.kDutyCycle);
					break;
				case Position:
					set(canEncoder.getPosition(), ControlType.kPosition);
					break;
				case Velocity:
					set(0, ControlType.kVelocity);
					break;
				case Voltage:
					set(0, ControlType.kVoltage);
					break;
				default:
					set(0, ControlType.kDutyCycle);
					break;
			}
		}
	}

	@Override
	public void setSetpoint(double setpoint) {
		ControlType controlType = getControlType();
		switch (controlType) {
			case kDutyCycle:
				set(setpoint, controlType);
				break;
			case kVelocity:
				set(setpoint, controlType);
				break;
			case kVoltage:
				set(setpoint, controlType);
				break;
			case kPosition:
				set(setpoint, controlType);
				break;
			default:
				break;
		}
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
		switch (getControlType()) {
			case kDutyCycle:
				return MCControlMode.PercentOut;
			case kVelocity:
				return MCControlMode.Velocity;
			case kVoltage:
				return MCControlMode.Voltage;
			case kPosition:
				return MCControlMode.Position;
			default:
				return MCControlMode.Disabled;
		}
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
