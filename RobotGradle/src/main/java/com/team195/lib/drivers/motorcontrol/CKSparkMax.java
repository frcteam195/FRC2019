package com.team195.lib.drivers.motorcontrol;

import com.revrobotics.*;
import com.team195.frc2019.constants.CalConstants;
import com.team195.frc2019.constants.Constants;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.reporters.DiagnosticMessage;
import com.team195.frc2019.reporters.MessageLevel;
import com.team195.lib.util.CachedValue;

import java.util.ArrayList;
import java.util.Optional;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import java.util.function.Function;

public class CKSparkMax extends CANSparkMax implements TuneableMotorController {
	private int currentSelectedSlot = 0;
	public final CANPIDController canPIDController;
	public final CANEncoder canEncoder;
	private double prevOutput = Double.MIN_VALUE;
	private double setpoint = 0;

	private ArrayList<Consumer<Void>> mUserConfigArray = new ArrayList<>();
	private ReentrantLock configArrLock = new ReentrantLock();
	private final Configuration mCurrentConfig;

//	private Configuration fastMasterConfig = new Configuration(5, 5, 20, 50);
//	private Configuration normalMasterConfig = new Configuration(10, 10, 20, 50);
//	private Configuration normalSlaveConfig = new Configuration(10, 100, 100, 100);

	private static final Configuration fastMasterConfig = new Configuration(5, 5, 10, 10);
	private static final Configuration normalMasterConfig = new Configuration(10, 10, 10, 10);
	private static final Configuration normalSlaveConfig = new Configuration(10, 100, 100, 100);

	private final PDPBreaker motorBreaker;

	private CachedValue<Double> mOutputCurrentCached;
	private CachedValue<Double> mOutputPercentCached;
	private CachedValue<Double> mInputVoltageCached;
	private CachedValue<Short> mFaultsCached;

	private MCControlMode currentControlMode = MCControlMode.PercentOut;

	private CKSparkMax(int deviceID, MotorType type, PDPBreaker breakerCurrent, Configuration deviceConfig) {
		super(deviceID, type);
//		restoreFactoryDefaults();
		motorBreaker = breakerCurrent;
		canPIDController = getPIDController();
		canEncoder = getEncoder();
		doDefaultConfig(deviceConfig);
		mCurrentConfig = deviceConfig;
		initCachedValues();
	}

	public CKSparkMax(int deviceID, MotorType type, boolean fastMaster, PDPBreaker breakerCurrent) {
		this(deviceID, type, breakerCurrent, fastMaster ? fastMasterConfig : normalMasterConfig);
		canPIDController.setOutputRange(-1, 1);
		setBrakeCoastMode(MCNeutralMode.Coast);
		burnFlash();
	}

	public CKSparkMax(int deviceID, MotorType type, CANSparkMax masterSpark, PDPBreaker breakerCurrent, boolean invert) {
		this(deviceID, type, breakerCurrent, normalSlaveConfig);
//		addConfigStatement((t) -> follow(masterSpark, invert));
		follow(masterSpark, invert);
		setBrakeCoastMode(MCNeutralMode.valueOf(masterSpark.getIdleMode()));
		burnFlash();
	}

	private void initCachedValues() {
		mOutputCurrentCached = new CachedValue<>(20, (t) -> super.getOutputCurrent());
		mOutputPercentCached = new CachedValue<>(20, (t) -> super.getAppliedOutput());
		mInputVoltageCached = new CachedValue<>(100, (t) -> super.getBusVoltage());
		mFaultsCached = new CachedValue<>(50, (t) -> super.getFaults());
	}

	private void doDefaultConfig(Configuration config) {
		//Fix encoder transient 0s which cause issues with all kinds of motion code
		setCANTimeout(500);

		setControlFramePeriodMs(config.CONTROL_FRAME_PERIOD_MS);
		runSparkMAXFunctionWithRetry((t) -> setPeriodicFramePeriod(PeriodicFrame.kStatus0, config.STATUS_FRAME_0_MS));
		runSparkMAXFunctionWithRetry((t) -> setPeriodicFramePeriod(PeriodicFrame.kStatus1, config.STATUS_FRAME_1_MS));
		runSparkMAXFunctionWithRetry((t) -> setPeriodicFramePeriod(PeriodicFrame.kStatus2, config.STATUS_FRAME_2_MS));
		runSparkMAXFunctionWithRetry((t) -> setSmartCurrentLimit(motorBreaker.value * 2));
		runSparkMAXFunctionWithRetry((t) -> enableVoltageCompensation(CalConstants.kDriveDefaultVoltageCompensationSetpoint));
		runSparkMAXFunctionWithRetry((t) -> setOpenLoopRampRate(CalConstants.kDriveDefaultOpenLoopRampRate));
		set(MCControlMode.PercentOut, 0, 0, 0);
	}

	public void setEncoderFactor(double conversionFactor) {
		runSparkMAXFunctionWithRetry((t) -> canEncoder.setPositionConversionFactor(conversionFactor));
		runSparkMAXFunctionWithRetry((t) -> canEncoder.setVelocityConversionFactor(conversionFactor));
	}

	@Deprecated
	public void set(double DO_NOT_USE) {
		set(MCControlMode.PercentOut, DO_NOT_USE, 0, 0);
	}

	@Override
	public void set(MCControlMode controlMode, double demand, int slotIdx, double arbitraryFeedForward) {
		if (currentSelectedSlot != slotIdx)
			setPIDGainSlot(slotIdx);

		//TODO: Remove if not necessary after testing position units
		demand = convertDemandToNativeUnits(controlMode, demand);

		if (demand + arbitraryFeedForward != prevOutput || currentSelectedSlot != slotIdx || controlMode != currentControlMode) {
			currentControlMode = controlMode;
			setpoint = demand;

			canPIDController.setReference(demand, controlMode.Rev(), slotIdx, arbitraryFeedForward, CANPIDController.ArbFFUnits.kPercentOut);
//			final double finalDemand = demand;
//			runSparkMAXFunctionWithRetry((t) -> canPIDController.setReference(finalDemand, controlMode.Rev(), slotIdx, arbitraryFeedForward));

			prevOutput = demand + arbitraryFeedForward;
		}
	}

	public void addConfigStatement(Consumer<Void> function) {
		//Run the command
		function.accept(null);
		//Add it to the array of userconfig commands
		try {
			if (configArrLock.tryLock(100, TimeUnit.MILLISECONDS)) {
				try {
					mUserConfigArray.add(function);
				} catch (Exception ex) {
					ConsoleReporter.report(ex);
				} finally {
					configArrLock.unlock();
				}
			}
		} catch (Exception ex) {
			ConsoleReporter.report(ex);
		}
	}

	public void runUserConfig() {
		try {
			if (configArrLock.tryLock(500, TimeUnit.MILLISECONDS)) {
				try {
					mUserConfigArray.forEach((f) -> f.accept(null));
				} catch (Exception ex) {
					ConsoleReporter.report(ex);
				} finally {
					configArrLock.unlock();
				}
			}
		} catch (Exception ex) {
			ConsoleReporter.report(ex);
		}
	}

	public synchronized void setMinimumSetpointOutput(double minSetpointOutput) {
		runSparkMAXFunctionWithRetry((t) -> canPIDController.setSmartMotionMinOutputVelocity(minSetpointOutput, currentSelectedSlot));
	}

	public synchronized void setAllowedClosedLoopError(double allowedClosedLoopError) {
		runSparkMAXFunctionWithRetry((t) -> canPIDController.setSmartMotionAllowedClosedLoopError(allowedClosedLoopError, currentSelectedSlot));
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
	public double getSetpoint() {
		return setpoint;
	}

	@Override
	public double getVelocityRPMTimeConversionFactor() {
		return 1;
	}

	@Override
	public double getNativeUnitsOutputRange() {
		return 1.0;
	}


	@Override
	public void setPIDF(double kP, double kI, double kD, double kF) {
		runSparkMAXFunctionWithRetry((t) -> canPIDController.setP(kP, currentSelectedSlot));
		runSparkMAXFunctionWithRetry((t) -> canPIDController.setI(kI, currentSelectedSlot));
		runSparkMAXFunctionWithRetry((t) -> canPIDController.setD(kD, currentSelectedSlot));
		runSparkMAXFunctionWithRetry((t) -> canPIDController.setFF(kF, currentSelectedSlot));
	}

	@Override
	public void setDFilter(double dFilter) {
		runSparkMAXFunctionWithRetry((t) -> canPIDController.setDFilter(dFilter, currentSelectedSlot));
	}

	@Override
	public void setIZone(double iZone) {
		runSparkMAXFunctionWithRetry((t) -> canPIDController.setIZone(iZone, currentSelectedSlot));
	}

	@Override
	public void setMCIAccum(double iAccum) {
		runSparkMAXFunctionWithRetry((t) -> canPIDController.setIAccum(iAccum));
	}

	@Override
	public void setMaxIAccum(double maxIAccum) {
		runSparkMAXFunctionWithRetry((t) -> canPIDController.setIMaxAccum(maxIAccum, currentSelectedSlot));
	}

	@Override
	public void setMCOpenLoopRampRate(double rampRate) {
		runSparkMAXFunctionWithRetry((t) -> setOpenLoopRampRate(rampRate));
	}

	@Override
	public void setMCClosedLoopRampRate(double rampRate) {
		runSparkMAXFunctionWithRetry((t) -> setClosedLoopRampRate(rampRate));
	}

	@Override
	public synchronized void setMotionParameters(double cruiseVel, double cruiseAccel) {
		runSparkMAXFunctionWithRetry((t) -> canPIDController.setSmartMotionMaxVelocity(cruiseVel, currentSelectedSlot));
		runSparkMAXFunctionWithRetry((t) -> canPIDController.setSmartMotionMaxAccel(cruiseAccel, currentSelectedSlot));
	}

	@Override
	public void setPIDGainSlot(int slotIdx) {
		setCurrentSelectedSlot(slotIdx);
	}

	@Override
	public void setBrakeCoastMode(MCNeutralMode neutralMode) {
		runSparkMAXFunctionWithRetry((t) -> setIdleMode(neutralMode.Rev()));
	}

	@Override
	public synchronized void setEncoderPosition(double position) {
		runSparkMAXFunctionWithRetry((t) -> canEncoder.setPosition(position));
	}

	@Override
	public void setCurrentLimit(int currentLimit) {
		runSparkMAXFunctionWithRetry((t) -> setSmartCurrentLimit(currentLimit));
	}

	@Override
	public void writeToFlash() {
		burnFlash();
	}

	@Override
	public void disableSoftLimits() {

	}

	@Override
	public boolean getForwardLimitValue() {
		return false;
	}

	@Override
	public boolean getReverseLimitValue() {
		return false;
	}

	@Override
	public boolean getForwardLimitRisingEdge() {
		return false;
	}

	@Override
	public boolean getReverseLimitRisingEdge() {
		return false;
	}

	@Override
	public boolean getForwardLimitFallingEdge() {
		return false;
	}

	@Override
	public boolean getReverseLimitFallingEdge() {
		return false;
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
				case MotionMagic:
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
				return getAppliedOutput();
			case kPosition:
			case kSmartMotion:
				return canEncoder.getPosition();
			default:
				return 0;
		}
	}

	@Override
	public double getMCIAccum() {
		return canPIDController.getIAccum();
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
	public synchronized DiagnosticMessage hasMotorControllerReset() {
		//TODO: Verify this works in SparkMax API
		if (getFault(FaultID.kHasReset)) {
			ConsoleReporter.report("Spark Max ID " + getDeviceId() + " has reset!", MessageLevel.DEFCON1);

			runSparkMAXFunctionWithRetry((t) -> clearFaults());
			//Do full reconfigure on SparkMAX after reset due to it's unreliable return after a breaker trip
			//Don't do this now that follow mode is fixed
//			doDefaultConfig(mCurrentConfig);
//			runUserConfig();

			return new DiagnosticMessage("SparkMax" + getDeviceId() + "ResetHasOccurred");
		}
		return DiagnosticMessage.NO_MSG;
	}

	@Override
	public boolean getFault(FaultID faultID) {
		short val = (short) (getFaults() & (1 << faultID.value));
		return val != 0;
	}

	@Override
	public short getFaults() {
		return mFaultsCached.getValue();
	}

	@Override
	public double getBusVoltage() {
		return mInputVoltageCached.getValue();
	}

	@Override
	public double getAppliedOutput() {
		return mOutputPercentCached.getValue();
	}

	@Override
	public double getOutputCurrent() {
		return mOutputCurrentCached.getValue();
	}

	private synchronized void runSparkMAXFunctionWithRetry(Function<Void, CANError> sparkMAXCall) {
		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = sparkMAXCall.apply(null) == CANError.kOK;
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to set parameter SparkMAX " + getDeviceId() + " !!!!!!", MessageLevel.DEFCON1);
	}

	@Override
	public MCControlMode getMotionControlMode() {
		return currentControlMode;
	}

	public ControlType getControlType() {
		Optional<Integer> o = getParameterInt(ConfigParameter.kCtrlType);
		if (o.isEmpty())
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
			case 4:
				return ControlType.kSmartMotion;
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
