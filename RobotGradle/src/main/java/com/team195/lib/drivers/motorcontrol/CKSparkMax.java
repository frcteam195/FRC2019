package com.team195.lib.drivers.motorcontrol;

import com.revrobotics.*;
import com.team195.frc2019.Constants;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.reporters.DiagnosticMessage;
import com.team195.frc2019.reporters.MessageLevel;
import com.team195.lib.util.ThreadRateControl;

import java.util.Optional;

public class CKSparkMax extends CANSparkMax implements TuneableMotorController {
	private int currentSelectedSlot = 0;
	private final CANPIDController canPIDController;
	private final CANEncoder canEncoder;
	private double prevOutput = Double.MIN_VALUE;
	private double encoderOffset = 0;

//	private Configuration fastMasterConfig = new Configuration(5, 5, 20, 50);
//	private Configuration normalMasterConfig = new Configuration(10, 10, 20, 50);
//	private Configuration normalSlaveConfig = new Configuration(10, 100, 100, 100);

	private Configuration fastMasterConfig = new Configuration(5, 5, 10, 10);
	private Configuration normalMasterConfig = new Configuration(10, 10, 10, 10);
	private Configuration normalSlaveConfig = new Configuration(10, 100, 100, 100);

	private double voltageCompensation = 12;
	private final PDPBreaker motorBreaker;

	private double prevMotionVelocitySetpoint = 0;
	private double minSetpointOutput = 0;
	private double allowedClosedLoopError = 0.1;
	private double lastNonZeroEncPosition = Double.longBitsToDouble(0x1L);
	private double lastNonZeroEncVelocity = Double.longBitsToDouble(0x1L);
	private MCControlMode currentControlMode = MCControlMode.PercentOut;

	private Thread motionMagicControlThread;
	private double motionMagicDemand = 0;
	private int motionMagicSlotIdx = 0;
	private double motionMagicArbitraryFF = 0;
	private double motionMagicMaxVelocity = 0;
	private double motionMagicMaxAccel = 0;

	private double speedNew = 0;
	private double decelVelocity = 0;

	public CKSparkMax(int deviceID, MotorType type, boolean fastMaster, PDPBreaker breakerCurrent) {
		super(deviceID, type);
		motorBreaker = breakerCurrent;
		canPIDController = getPIDController();
		canEncoder = getEncoder();
		canPIDController.setOutputRange(-1, 1);
		doDefaultConfig(fastMaster ? fastMasterConfig : normalMasterConfig);
		setMinimumSetpointOutput(25);

	}

	public CKSparkMax(int deviceID, MotorType type, CANSparkMax masterSpark, PDPBreaker breakerCurrent) {
		super(deviceID, type);
		motorBreaker = breakerCurrent;
		canPIDController = getPIDController();
		canEncoder = getEncoder();
		doDefaultConfig(normalSlaveConfig);
		follow(masterSpark);

		motionMagicControlThread = new Thread(() -> {});
	}

	private void startMotionMagicControlThread() {
		if (motionMagicControlThread != null) {
			if (!motionMagicControlThread.isAlive()) {
				resetMotionMagicLambda();
				motionMagicControlThread.start();
			}
		} else {
			resetMotionMagicLambda();
			motionMagicControlThread.start();
		}
	}

	private void resetMotionMagicLambda() {
		//Instantiate Motion Magic Thread
		motionMagicControlThread = new Thread(() -> {
			ThreadRateControl trc = new ThreadRateControl();
			trc.start();
			while (currentControlMode == MCControlMode.MotionMagic) {
				double demandStep = generateMotionMagicValue(getPosition(), motionMagicDemand, getVelocity(), trc.getDt());
				prevMotionVelocitySetpoint = demandStep;
				demandStep = Math.abs(demandStep) < minSetpointOutput ? 0 : demandStep;
				canPIDController.setReference(demandStep, MCControlMode.Velocity.Rev(), motionMagicSlotIdx, motionMagicArbitraryFF * voltageCompensation);
				trc.doRateControl(10);
			}
		});
	}

	private void doDefaultConfig(Configuration config) {
		//Fix encoder transient 0s which cause issues with all kinds of motion code
		setCANTimeout(500);

		setControlFramePeriod(config.CONTROL_FRAME_PERIOD_MS);
		boolean setSucceeded;
		int retryCounter = 0;
		do {
			setSucceeded = setPeriodicFramePeriod(PeriodicFrame.kStatus0, config.STATUS_FRAME_0_MS) == CANError.kOK;
			setSucceeded &= setPeriodicFramePeriod(PeriodicFrame.kStatus1, config.STATUS_FRAME_1_MS) == CANError.kOK;
			setSucceeded &= setPeriodicFramePeriod(PeriodicFrame.kStatus2, config.STATUS_FRAME_2_MS) == CANError.kOK;
			setSucceeded &= setSmartCurrentLimit(motorBreaker.value * 2) == CANError.kOK;
			//Erase previously stored values
			set(MCControlMode.PercentOut, 0, 0, 0);
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);
		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to config Spark Max " + getDeviceId() + " !!!!!!", MessageLevel.DEFCON1);
	}

	@Deprecated
	public void set(double DO_NOT_USE) {

	}

	@Override
	public void set(MCControlMode controlMode, double demand, int slotIdx, double arbitraryFeedForward) {
		if (currentSelectedSlot != slotIdx)
			setPIDGainSlot(slotIdx);

		//TODO: Remove if not necessary after testing position units
		demand = convertDemandToNativeUnits(controlMode, demand);

		if (demand + arbitraryFeedForward != prevOutput || currentSelectedSlot != slotIdx || controlMode != currentControlMode) {
			currentControlMode = controlMode;

			if (controlMode == MCControlMode.MotionMagic) {
				motionMagicDemand = demand;
				motionMagicSlotIdx = slotIdx;
				motionMagicArbitraryFF = arbitraryFeedForward;
				startMotionMagicControlThread();
			} else {
				boolean setSucceeded;
				int retryCounter = 1;

				do {
					System.out.println(getPosition());
					setSucceeded = canPIDController.setReference(demand, controlMode.Rev(), slotIdx, arbitraryFeedForward * voltageCompensation) == CANError.kOK;
				} while (!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

				if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
					ConsoleReporter.report("Failed to set output Spark Max " + getDeviceId() + " !!!!!!", MessageLevel.DEFCON1);
			}

			prevOutput = demand + arbitraryFeedForward;
		}
	}

	public double getMotionPositionSetpoint() {
		return motionMagicDemand;
	}

	public double getPrevMotionVelocitySetpoint() {
		return prevMotionVelocitySetpoint;
	}

	public synchronized void setMinimumSetpointOutput(double minSetpointOutput) {
		this.minSetpointOutput = minSetpointOutput;
	}

	public synchronized void setAllowedClosedLoopError(double allowedClosedLoopError) {
		this.allowedClosedLoopError = allowedClosedLoopError;
	}

	private double generateMotionMagicValue(double position, double setpoint, double speed, double dt) {
		double diffErr = setpoint - position;
		double sign = Math.signum(diffErr);
		diffErr = Math.abs(diffErr);
		if (motionMagicMaxVelocity == 0 || motionMagicMaxAccel == 0 || diffErr <= allowedClosedLoopError)
			return 0;

		speedNew = Math.abs(prevMotionVelocitySetpoint) + motionMagicMaxAccel * dt;
		speedNew = speedNew > motionMagicMaxVelocity ? motionMagicMaxVelocity : speedNew;
		double decelTime = Math.sqrt(diffErr * 2 / (motionMagicMaxAccel * 60));
		decelVelocity = decelTime * (motionMagicMaxAccel * 60);
		return sign * (speedNew <= decelVelocity ? speedNew : decelVelocity);
	}

	//Filter 0 due to issue with API returning transient 0s
	public double getPosition() {
		double encPos = canEncoder.getPosition();
		if (encPos == 0) {
			return lastNonZeroEncPosition;
		} else {
			setLastNonZeroEncPosition(encPos + encoderOffset);
			return lastNonZeroEncPosition;
		}
	}

	//Filter 0 due to issue with API returning transient 0s
	public double getVelocity() {
//		double encVel = canEncoder.getVelocity();
//		if (encVel == 0) {
//			return lastNonZeroEncVelocity;
//		} else {
//			lastNonZeroEncVelocity = encVel;
//			return lastNonZeroEncVelocity;
//		}
		return canEncoder.getVelocity();
	}

	private synchronized void setLastNonZeroEncPosition(double position) {
		lastNonZeroEncPosition = position;
	}

	//TODO: Remove
	public double getSpeedNew() {
		return speedNew;
	}

	//TODO: Remove
	public double getDecelVelocity() {
		return decelVelocity;
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

	@Override
	public double getNativeUnitsOutputRange() {
		return 1.0;
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
		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = canPIDController.setP(kP, currentSelectedSlot) == CANError.kOK;
			setSucceeded &= canPIDController.setI(kI, currentSelectedSlot) == CANError.kOK;
			setSucceeded &= canPIDController.setD(kD, currentSelectedSlot) == CANError.kOK;
			setSucceeded &= canPIDController.setFF(kF, currentSelectedSlot) == CANError.kOK;
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to set PID Gains Spark Max " + getDeviceId() + " !!!!!!", MessageLevel.DEFCON1);
	}

	@Override
	public void setIZone(double iZone) {
		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = canPIDController.setIZone(iZone, currentSelectedSlot) == CANError.kOK;
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to set IZone Spark Max " + getDeviceId() + " !!!!!!", MessageLevel.DEFCON1);
	}

	@Override
	public void setIAccum(double iAccum) {

	}

	@Override
	public void setMaxIAccum(double maxIAccum) {

	}

	@Override
	public void setMCOpenLoopRampRate(double rampRate) {
		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = setRampRate(rampRate) == CANError.kOK;
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to set Ramp rate Spark Max " + getDeviceId() + " !!!!!!", MessageLevel.DEFCON1);
	}

	@Override
	public void setMCClosedLoopRampRate(double rampRate) {
		setMCOpenLoopRampRate(rampRate);
	}

	@Override
	public synchronized void setMotionParameters(int cruiseVel, int cruiseAccel) {
		motionMagicMaxAccel = cruiseAccel;
		motionMagicMaxVelocity = cruiseVel;
	}

	@Override
	public void setPIDGainSlot(int slotIdx) {
		setCurrentSelectedSlot(slotIdx);
	}

	@Override
	public void setBrakeCoastMode(MCNeutralMode neutralMode) {
		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = setIdleMode(neutralMode.Rev()) == CANError.kOK;
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to set Ramp rate Spark Max " + getDeviceId() + " !!!!!!", MessageLevel.DEFCON1);
	}

	@Override
	public synchronized void setEncoderPosition(double position) {
		setEncoderOffset(position - getPosition());
		setLastNonZeroEncPosition(position == 0 ? Double.longBitsToDouble(0x1L) : position);
	}

	private synchronized void setEncoderOffset(double offset) {
		encoderOffset = offset;
	}

	private synchronized void setCurrentSelectedSlot(int slotIdx) {
		currentSelectedSlot = slotIdx;
	}

	@Override
	public synchronized void setControlMode(MCControlMode controlMode) {
		if (currentControlMode != controlMode)
		{
			currentControlMode = controlMode;
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
	public double getMCOutputCurrent() {
		return getOutputCurrent();
	}

	@Override
	public double getMCOutputPercent() {
		return getAppliedOutput();
	}

	@Override
	public int getMCID() {
		return getDeviceId();
	}

	@Override
	public double getMCInputVoltage() {
		return getBusVoltage();
	}

	@Override
	public double getMCOutputVoltage() {
		return getMCOutputPercent()*getMCInputVoltage();
	}

	@Override
	public boolean isEncoderPresent() {
		//TODO: Test
		return !getFault(FaultID.kSensorFault);
	}

	@Override
	public DiagnosticMessage hasMotorControllerReset() {
		//TODO: Verify this works in SparkMax API
		if (getFault(FaultID.kHasReset)) {
			ConsoleReporter.report("Spark Max ID " + getDeviceId() + " has reset!", MessageLevel.DEFCON1);

			boolean setSucceeded;
			int retryCounter = 0;

			do {
				setSucceeded = clearFaults() == CANError.kOK;
			} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

			if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
				ConsoleReporter.report("Failed to clear Spark Max ID " + getDeviceId() + " Reset !!!!!!", MessageLevel.DEFCON1);

			return new DiagnosticMessage("SparkMax" + getDeviceId() + "ResetHasOccurred");
		}
		return DiagnosticMessage.NO_MSG;
	}

	@Override
	public MCControlMode getMotionControlMode() {
		return currentControlMode;
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
