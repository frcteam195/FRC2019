package com.team195.lib.drivers.motorcontrol;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team195.frc2019.Constants;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.reporters.DiagnosticMessage;
import com.team195.frc2019.reporters.MessageLevel;
import com.team195.lib.util.ThreadRateControl;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;

public class CKTalonSRX implements TuneableMotorController {
	private final TalonSRX mTalonSRX;
	private int currentSelectedSlot = 0;
	private double[] mCLRampRate = new double[4];
	private int[] mMMAccel = new int[4];
	private int[] mMMVel = new int[4];
	private double prevOutput = Double.MIN_VALUE;
	private final PDPBreaker motorBreaker;
	private boolean prevForwardLimitVal = false;
	private boolean prevReverseLimitVal = false;

	private final Configuration fastMasterConfig = new Configuration(5, 5, 20);
	private final Configuration normalMasterConfig = new Configuration(10, 10, 20);
	private final Configuration normalSlaveConfig = new Configuration(10, 100, 100);


	private double prevMotionVelocitySetpoint = 0;
	private double minSetpointOutput = 0;
	private double allowedClosedLoopError = 0;
	private MCControlMode currentControlMode = MCControlMode.Disabled;  //Force an update

	private Thread motionVoodooArbFFControlThread;
	private double motionVoodooArbFFDemand = 0;
	public InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> motionVoodooArbFFLookup = new InterpolatingTreeMap<>();
	public double absoluteEncoderOffset = 0;

	private double speedNew = 0;

	private TalonSRX getRawTalonSRX() {
		return mTalonSRX;
	}

	public CKTalonSRX(int deviceId, boolean fastMaster, PDPBreaker breakerCurrent) {
		mTalonSRX = new TalonSRX(deviceId);
		mTalonSRX.configFactoryDefault();
		motorBreaker = breakerCurrent;
		doDefaultConfig(fastMaster ? fastMasterConfig : normalMasterConfig);
		set(MCControlMode.PercentOut, 0, 0, 0);
		setBrakeCoastMode(MCNeutralMode.Brake);
	}

	public CKTalonSRX(int deviceId, CKTalonSRX masterTalon, PDPBreaker breakerCurrent, boolean inverted) {
		mTalonSRX = new TalonSRX(deviceId);
		mTalonSRX.configFactoryDefault();
		motorBreaker = breakerCurrent;
		doDefaultConfig(normalSlaveConfig);
		setBrakeCoastMode(MCNeutralMode.Brake);
		mTalonSRX.follow(masterTalon.getRawTalonSRX());
		mTalonSRX.setInverted(inverted);
	}

	private void doDefaultConfig(Configuration config) {
		boolean setSucceeded;
		int retryCounter = 0;
		do {
			setSucceeded = mTalonSRX.clearStickyFaults(Constants.kLongCANTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mTalonSRX.setControlFramePeriod(ControlFrame.Control_3_General, config.CONTROL_FRAME_PERIOD_MS) == ErrorCode.OK;
			setSucceeded &= mTalonSRX.setStatusFramePeriod(StatusFrame.Status_1_General, config.STATUS_FRAME_GENERAL_1_MS, Constants.kLongCANTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mTalonSRX.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, config.STATUS_FRAME_FEEDBACK0_2_MS, Constants.kLongCANTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mTalonSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kLongCANTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mTalonSRX.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, Constants.kLongCANTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mTalonSRX.configVelocityMeasurementWindow(1, Constants.kLongCANTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mTalonSRX.configContinuousCurrentLimit(motorBreaker.value, Constants.kLongCANTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mTalonSRX.configPeakCurrentLimit(motorBreaker.value * 2, Constants.kLongCANTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mTalonSRX.configPeakCurrentDuration(getMSDurationForBreakerLimit(motorBreaker.value * 2, motorBreaker.value), Constants.kLongCANTimeoutMs) == ErrorCode.OK;
			mTalonSRX.enableCurrentLimit(true);
			setSucceeded &= mTalonSRX.configVoltageCompSaturation(12) == ErrorCode.OK;
			setSucceeded &= mTalonSRX.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mTalonSRX.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mTalonSRX.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, Constants.kLongCANTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mTalonSRX.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, Constants.kLongCANTimeoutMs) == ErrorCode.OK;
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);
		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to initialize Talon " + mTalonSRX.getDeviceID() + " !!!!!!", MessageLevel.DEFCON1);
	}

	public ErrorCode configForwardLimitSwitchSource(LimitSwitchSource type, LimitSwitchNormal normalOpenOrClose) {

		return mTalonSRX.configForwardLimitSwitchSource(type, normalOpenOrClose);
	}

	public void setSensorPhase(boolean inverted) {
		mTalonSRX.setSensorPhase(inverted);
	}

	public void setInverted(boolean inverted) {
		mTalonSRX.setInverted(inverted);
	}

	public void setInverted(InvertType invertType) {
		mTalonSRX.setInverted(invertType);
	}

	public ErrorCode configClosedloopRamp(double secondsFromNeutralToFull, int timeoutMs) {
		return configClosedloopRamp(secondsFromNeutralToFull, currentSelectedSlot, timeoutMs);
	}

	public ErrorCode configClosedloopRamp(double secondsFromNeutralToFull, int slotIdx, int timeoutMs) {
		setCurrentSlotCLRampRate(secondsFromNeutralToFull, slotIdx);
		return mTalonSRX.configClosedloopRamp(secondsFromNeutralToFull, timeoutMs);
	}

	public ErrorCode configMotionAcceleration(int sensorUnitsPer100msPerSec, int timeoutMs) {
		return configMotionAcceleration(sensorUnitsPer100msPerSec, currentSelectedSlot, timeoutMs);
	}

	public ErrorCode configMotionAcceleration(int sensorUnitsPer100msPerSec, int slotIdx, int timeoutMs) {
		setCurrentMMAccel(sensorUnitsPer100msPerSec, slotIdx);
		return mTalonSRX.configMotionAcceleration(sensorUnitsPer100msPerSec, timeoutMs);
	}

	public ErrorCode configMotionCruiseVelocity(int sensorUnitsPer100ms, int timeoutMs) {
		return configMotionCruiseVelocity(sensorUnitsPer100ms, currentSelectedSlot, timeoutMs);
	}

	public ErrorCode configMotionCruiseVelocity(int sensorUnitsPer100ms, int slotIdx, int timeoutMs) {
		setCurrentMMVel(sensorUnitsPer100ms, slotIdx);
		return mTalonSRX.configMotionCruiseVelocity(sensorUnitsPer100ms, timeoutMs);
	}

	public synchronized void setAbsoluteEncoderOffset(double offset) {
		absoluteEncoderOffset = offset;
	}

	/**
	 * Make sure you call this method before first use of set() to ensure CL ramp rate and PID gains are selected properly when using CKTalonSRX
	 * @param slotIdx Gain profile slot
	 * @param pidIdx PID ID, 0 for main, 1 for aux
	 */
	public void selectProfileSlot(int slotIdx, int pidIdx) {
		mTalonSRX.selectProfileSlot(slotIdx, pidIdx);
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
				ConsoleReporter.report("Failed to change Talon ID" + mTalonSRX.getDeviceID() +  " profile slot!!!", MessageLevel.DEFCON1);
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
		sb.append("General Status Frame 1: " + mTalonSRX.getStatusFramePeriod(StatusFrameEnhanced.Status_1_General, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 2: " + mTalonSRX.getStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 3: " + mTalonSRX.getStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 4: " + mTalonSRX.getStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 6: " + mTalonSRX.getStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 7: " + mTalonSRX.getStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 8: " + mTalonSRX.getStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 9: " + mTalonSRX.getStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 10: " + mTalonSRX.getStatusFramePeriod(StatusFrame.Status_10_Targets, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 11: " + mTalonSRX.getStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 12: " + mTalonSRX.getStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 13: " + mTalonSRX.getStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 14: " + mTalonSRX.getStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, Constants.kLongCANTimeoutMs) + "\r\n");
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

	@Override
	public void set(MCControlMode controlMode, double demand, int slotIdx, double arbitraryFeedForward) {
		if (currentSelectedSlot != slotIdx)
			selectProfileSlot(slotIdx, 0);

		if (demand + arbitraryFeedForward != prevOutput || currentSelectedSlot != slotIdx || controlMode != currentControlMode) {
			currentControlMode = controlMode;

			if (controlMode == MCControlMode.MotionVoodooArbFF) {
				motionVoodooArbFFDemand = demand;
				startMotionVoodooArbFFControlThread();
			} else {
				demand = convertDemandToNativeUnits(controlMode, demand);
				mTalonSRX.set(controlMode.CTRE(), demand, DemandType.ArbitraryFeedForward, arbitraryFeedForward);
				prevOutput = demand + arbitraryFeedForward;
			}
		}
	}

	private void startMotionVoodooArbFFControlThread() {
		if (motionVoodooArbFFControlThread != null) {
			if (!motionVoodooArbFFControlThread.isAlive()) {
				resetMotionVoodooArbFFLambda();
				motionVoodooArbFFControlThread.start();
			}
		} else {
			resetMotionVoodooArbFFLambda();
			motionVoodooArbFFControlThread.start();
		}
	}

	private void resetMotionVoodooArbFFLambda() {
		//Instantiate Motion Magic Thread
		motionVoodooArbFFControlThread = new Thread(() -> {
			ThreadRateControl trc = new ThreadRateControl();
			trc.start();
			while (currentControlMode == MCControlMode.MotionVoodooArbFF) {
				double demandStep = generateMotionVoodooArbFFValue(getPosition(), motionVoodooArbFFDemand, getVelocity(), trc.getDt());
				prevMotionVelocitySetpoint = demandStep;
				demandStep = Math.abs(demandStep) < minSetpointOutput ? 0 : demandStep;
				double arbFF = motionVoodooArbFFLookup.getInterpolated(new InterpolatingDouble(getPosition() - absoluteEncoderOffset)).value;
				mTalonSRX.set(ControlMode.Velocity, convertDemandToNativeUnits(MCControlMode.MotionVoodooArbFF, demandStep), DemandType.ArbitraryFeedForward, arbFF);
//				ConsoleReporter.report("ArbFF: " + arbFF + ", OutputDC: " + getMCOutputPercent() + ", Pos: " + getPosition() + ", Spd: " + demandStep);
				trc.doRateControl(10);
			}
		});
	}

	private double generateMotionVoodooArbFFValue(double position, double setpoint, double speed, double dt) {
		double motionVoodooArbFFVelocity = convertNativeUnitsToRPM(mMMVel[currentSelectedSlot]);
		double motionVoodooArbFFMaxAccel = convertNativeUnitsToRPM(mMMAccel[currentSelectedSlot]);

		double diffErr = setpoint - position;
		double diffSign = Math.copySign(1.0, diffErr);
		diffErr = Math.abs(diffErr);

		if (motionVoodooArbFFVelocity == 0 || motionVoodooArbFFMaxAccel == 0 || diffErr <= allowedClosedLoopError)
			return 0;

		double decelVelocity = Math.sqrt(diffErr * 2.0 / (motionVoodooArbFFMaxAccel * 60.0)) * (motionVoodooArbFFVelocity * 60.0);

		speedNew = prevMotionVelocitySetpoint + (motionVoodooArbFFMaxAccel * dt * diffSign);
		double absoluteSpeed = Math.abs(speedNew);
		speedNew = absoluteSpeed > motionVoodooArbFFVelocity ? Math.copySign(motionVoodooArbFFVelocity, speedNew) : speedNew;

		speedNew = absoluteSpeed <= decelVelocity ? speedNew : Math.copySign(decelVelocity, speedNew);
		speedNew = absoluteSpeed <= minSetpointOutput ? 0 : speedNew;
		return speedNew;
	}

	@Override
	public void setPIDF(double kP, double kI, double kD, double kF) {
		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = mTalonSRX.config_kP(currentSelectedSlot, kP, Constants.kCANTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mTalonSRX.config_kI(currentSelectedSlot, kI, Constants.kCANTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mTalonSRX.config_kD(currentSelectedSlot, kD, Constants.kCANTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mTalonSRX.config_kF(currentSelectedSlot, kF, Constants.kCANTimeoutMs) == ErrorCode.OK;
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to set PID Gains Talon " + mTalonSRX.getDeviceID() + " !!!!!!", MessageLevel.DEFCON1);
	}

	@Override
	public void setDFilter(double dFilter) {

	}

	@Override
	public void setIZone(double iZone) {
		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = mTalonSRX.config_IntegralZone(currentSelectedSlot, (int)iZone, Constants.kCANTimeoutMs) == ErrorCode.OK;
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to set IZone Talon " + mTalonSRX.getDeviceID() + " !!!!!!", MessageLevel.DEFCON1);
	}

	@Override
	public void setMCIAccum(double iAccum) {
		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = mTalonSRX.setIntegralAccumulator(iAccum, 0, Constants.kCANTimeoutMs) == ErrorCode.OK;
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to set I Accum Talon " + mTalonSRX.getDeviceID() + " !!!!!!", MessageLevel.DEFCON1);
	}

	@Override
	public void setMaxIAccum(double maxIAccum) {
		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = mTalonSRX.configMaxIntegralAccumulator(currentSelectedSlot, maxIAccum, Constants.kCANTimeoutMs) == ErrorCode.OK;
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to set Max I Accum Talon " + mTalonSRX.getDeviceID() + " !!!!!!", MessageLevel.DEFCON1);
	}

	@Override
	public void setMCOpenLoopRampRate(double rampRate) {
		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = mTalonSRX.configOpenloopRamp(rampRate, Constants.kCANTimeoutMs) == ErrorCode.OK;
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to set MC Closed Loop Ramp Rate Talon " + mTalonSRX.getDeviceID() + " !!!!!!", MessageLevel.DEFCON1);
	}

	@Override
	public void setMCClosedLoopRampRate(double rampRate) {
		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = configClosedloopRamp(rampRate, Constants.kCANTimeoutMs) == ErrorCode.OK;
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to set MC Closed Loop Ramp Rate Talon " + mTalonSRX.getDeviceID() + " !!!!!!", MessageLevel.DEFCON1);
	}

	@Override
	public void setMotionParameters(int cruiseVel, int cruiseAccel) {
		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = configMotionCruiseVelocity(convertRPMToNativeUnits(cruiseVel), Constants.kCANTimeoutMs) == ErrorCode.OK;
			setSucceeded &= configMotionAcceleration(convertRPMToNativeUnits(cruiseAccel), Constants.kCANTimeoutMs) == ErrorCode.OK;
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to set Motion Params Talon " + mTalonSRX.getDeviceID() + " !!!!!!", MessageLevel.DEFCON1);
	}

	@Override
	public void setPIDGainSlot(int slotIdx) {
		if (currentSelectedSlot != slotIdx)
			selectProfileSlot(slotIdx, 0);
	}

	@Override
	public void setBrakeCoastMode(MCNeutralMode neutralMode) {
		mTalonSRX.setNeutralMode(neutralMode.CTRE());
	}

	@Override
	public void setEncoderPosition(double position) {
		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = mTalonSRX.setSelectedSensorPosition(convertRotationsToNativeUnits(position), 0, Constants.kCANTimeoutMs) == ErrorCode.OK;
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to set encoder position Talon " + mTalonSRX.getDeviceID() + " !!!!!!", MessageLevel.DEFCON1);
	}

	@Override
	public void writeToFlash() {

	}

	@Override
	public boolean getForwardLimitValue() {
		return mTalonSRX.getSensorCollection().isFwdLimitSwitchClosed();
	}

	@Override
	public boolean getReverseLimitValue() {
		return mTalonSRX.getSensorCollection().isRevLimitSwitchClosed();
	}

	@Override
	public boolean getForwardLimitRisingEdge() {
		boolean currentInput = mTalonSRX.getSensorCollection().isFwdLimitSwitchClosed();
		boolean retVal = (currentInput != prevForwardLimitVal) && currentInput;
		prevForwardLimitVal = currentInput;
		return retVal;
	}

	@Override
	public boolean getReverseLimitRisingEdge() {
		boolean currentInput = mTalonSRX.getSensorCollection().isRevLimitSwitchClosed();
		boolean retVal = (currentInput != prevReverseLimitVal) && currentInput;
		prevReverseLimitVal = currentInput;
		return retVal;
	}

	@Override
	public boolean getForwardLimitFallingEdge() {
		boolean currentInput = mTalonSRX.getSensorCollection().isFwdLimitSwitchClosed();
		boolean retVal = (currentInput != prevForwardLimitVal) && !currentInput;
		prevForwardLimitVal = currentInput;
		return retVal;
	}

	@Override
	public boolean getReverseLimitFallingEdge() {
		boolean currentInput = mTalonSRX.getSensorCollection().isRevLimitSwitchClosed();
		boolean retVal = (currentInput != prevReverseLimitVal) && !currentInput;
		prevReverseLimitVal = currentInput;
		return retVal;
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
				case MotionVoodooArbFF:
					set(controlMode, getPosition(), currentSelectedSlot, 0);
					break;
				default:
					break;
			}
		}
	}

	@Override
	public MCControlMode getMotionControlMode() {
		return currentControlMode;
	}

	@Override
	public void setSetpoint(double setpoint) {
		set(getMotionControlMode(), setpoint, currentSelectedSlot, 0);
	}

	@Override
	public double getActual() {
		switch (currentControlMode) {
			case PercentOut:
//				return getMotorOutputPercent();
			case Position:
			case MotionMagic:
			case MotionVoodooArbFF:
				return getPosition();
			case Velocity:
				return getVelocity();
			case Current:
				return mTalonSRX.getOutputCurrent();
			default:
				return 0;
		}
	}

	@Override
	public double getSetpoint() {
		switch (currentControlMode) {
			case PercentOut:
//				return getMotorOutputPercent();
			case Position:
			case MotionMagic:
				return convertNativeUnitsToRotations(mTalonSRX.getClosedLoopTarget());
			case MotionVoodooArbFF:
				return motionVoodooArbFFDemand;
			case Velocity:
				return convertNativeUnitsToRPM((int)mTalonSRX.getClosedLoopTarget());
			case Current:
				return mTalonSRX.getClosedLoopTarget();
			default:
				return 0;
		}
	}

	@Override
	public double getMCOutputCurrent() {
		return mTalonSRX.getOutputCurrent();
	}

	@Override
	public double getMCOutputPercent() {
		return mTalonSRX.getMotorOutputPercent();
	}

	@Override
	public int getMCID() {
		return mTalonSRX.getDeviceID();
	}

	@Override
	public boolean isEncoderPresent() {
		return mTalonSRX.getSensorCollection().getPulseWidthRiseToRiseUs() != 0;
	}

	@Override
	public double getMCInputVoltage() {
		return mTalonSRX.getBusVoltage();
	}

	@Override
	public double getMCOutputVoltage() {
		return mTalonSRX.getMotorOutputVoltage();
	}

	@Override
	public double getPosition() {
		return convertNativeUnitsToRotations(mTalonSRX.getSelectedSensorPosition());
	}

	@Override
	public double getVelocity() {
		return convertNativeUnitsToRPM(mTalonSRX.getSelectedSensorVelocity());
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
	public double getNativeUnitsOutputRange() {
		return 1023.0;
	}

	@Override
	public double getMCIAccum() {
		return mTalonSRX.getIntegralAccumulator();
	}

	@Override
	public DiagnosticMessage hasMotorControllerReset() {
		if (mTalonSRX.hasResetOccurred()) {

			ConsoleReporter.report("Talon ID " + mTalonSRX.getDeviceID() + " has reset!", MessageLevel.DEFCON1);

			boolean setSucceeded;
			int retryCounter = 0;

			do {
				setSucceeded = mTalonSRX.clearStickyFaults(Constants.kCANTimeoutMs) == ErrorCode.OK;
			} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

			if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
				ConsoleReporter.report("Failed to clear Talon ID " + mTalonSRX.getDeviceID() + " Reset !!!!!!", MessageLevel.DEFCON1);

			return new DiagnosticMessage("Talon" + mTalonSRX.getDeviceID() + "ResetHasOccurred");
		}

		return DiagnosticMessage.NO_MSG;
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

