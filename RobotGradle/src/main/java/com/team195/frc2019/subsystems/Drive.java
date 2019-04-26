package com.team195.frc2019.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.team195.frc2019.constants.CalConstants;
import com.team195.frc2019.constants.DeviceIDConstants;
import com.team195.frc2019.constants.TestConstants;
import com.team195.frc2019.loops.ILooper;
import com.team195.frc2019.loops.Loop;
import com.team195.frc2019.planners.DriveMotionPlanner;
import com.team195.frc2019.RobotState;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.reporters.MessageLevel;
import com.team195.frc2019.reporters.ReflectingLogDataGenerator;
import com.team195.lib.drivers.CKDoubleSolenoid;
import com.team195.lib.drivers.CKIMU;
import com.team195.lib.drivers.NavX;
import com.team195.lib.drivers.motorcontrol.*;
import com.team195.lib.util.CachedValue;
import com.team195.lib.util.MotorDiagnostics;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.stream.Collectors;

public class Drive extends Subsystem {

	private static final int kLowGearVelocityControlSlot = 0;
	private static Drive mInstance = new Drive();
	private final CKSparkMax mLeftMaster, mRightMaster, mLeftSlaveA, mRightSlaveA, mLeftSlaveB, mRightSlaveB;
	private final CKDoubleSolenoid mPTOShifter;
	private DriveControlState mDriveControlState;
	private CKIMU mGyro;
	private PeriodicIO mPeriodicIO;
	private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;
	private ReflectingLogDataGenerator<PeriodicIO> mLogDataGenerator = new ReflectingLogDataGenerator<>(PeriodicIO.class);
	private DriveMotionPlanner mMotionPlanner;
	private Rotation2d mGyroOffset = Rotation2d.identity();
	private boolean mOverrideTrajectory = false;

	private AtomicBoolean mIsBrakeMode = new AtomicBoolean(false);
	private AtomicBoolean mForceBrakeUpdate = new AtomicBoolean(false);
	private boolean mPrevBrakeMode;

	private final CachedValue<Boolean> mLeftDriveEncoderPresent;
	private final CachedValue<Boolean> mRightDriveEncoderPresent;
	private final CachedValue<Boolean> mGyroPresent;

	private static final Elevator mElevator = Elevator.getInstance();

	private final Loop mLoop = new Loop() {
		@Override
		public void onFirstStart(double timestamp) {
			synchronized (Drive.this) {

			}
		}

		@Override
		public void onStart(double timestamp) {
			synchronized (Drive.this) {
				setOpenLoop(new DriveSignal(0, 0));
				if (mDriveControlState == DriveControlState.OPEN_LOOP) {
					setBrakeMode(false);
				}
				else {
					setBrakeMode(true);
				}
			}
		}

		@Override
		public void onLoop(double timestamp) {
			synchronized (Drive.this) {
				switch (mDriveControlState) {
					case OPEN_LOOP:
						break;
					case PATH_FOLLOWING:
						updatePathFollower();
						break;
					case CLIMB:
					case OPEN_LOOP_AUTOMATED:
					case VELOCITY:
						break;
					default:
						ConsoleReporter.report("Unexpected drive control state: " + mDriveControlState, MessageLevel.DEFCON1);
						break;
				}
			}
		}

		@Override
		public void onStop(double timestamp) {
			stop();
		}

		@Override
		public String getName() {
			return "Drive";
		}
	};

	private Drive() {
		mPeriodicIO = new PeriodicIO();

		mLeftMaster = new CKSparkMax(DeviceIDConstants.kLeftDriveMasterId, CANSparkMaxLowLevel.MotorType.kBrushless, true, PDPBreaker.B40A);
		mLeftMaster.addConfigStatement((t) -> mLeftMaster.setInverted(false));
		mLeftMaster.addConfigStatement((t) -> mLeftMaster.setPIDF(CalConstants.kDriveLowGearVelocityKp, CalConstants.kDriveLowGearVelocityKi, CalConstants.kDriveLowGearVelocityKd, CalConstants.kDriveLowGearVelocityKf));
		mLeftMaster.addConfigStatement((t) -> mLeftMaster.setDFilter(CalConstants.kDriveLowGearVelocityDFilter));
		mLeftMaster.addConfigStatement((t) -> mLeftMaster.setMotionParameters(CalConstants.kDriveLowGearPositionCruiseVel, CalConstants.kDriveLowGearPositionAccel));
		mLeftMaster.addConfigStatement((t) -> mLeftMaster.setCurrentLimit(CalConstants.kDriveLowGearCurrentLim));
		mLeftMaster.addConfigStatement((t) -> mLeftMaster.setClosedLoopRampRate(CalConstants.kDriveVoltageRampRate));
		mLeftMaster.setEncoderFactor(1);
		mLeftMaster.setAllowedClosedLoopError(0);
		mLeftMaster.writeToFlash();

		mLeftSlaveA = new CKSparkMax(DeviceIDConstants.kLeftDriveSlaveAId, CANSparkMaxLowLevel.MotorType.kBrushless, mLeftMaster, PDPBreaker.B40A, false);
		mLeftSlaveA.addConfigStatement((t) -> mLeftSlaveA.setCurrentLimit(CalConstants.kDriveLowGearCurrentLim));
		mLeftSlaveA.addConfigStatement((t) -> mLeftSlaveA.setClosedLoopRampRate(CalConstants.kDriveVoltageRampRate));
		mLeftSlaveA.writeToFlash();

		mLeftSlaveB = new CKSparkMax(DeviceIDConstants.kLeftDriveSlaveBId, CANSparkMaxLowLevel.MotorType.kBrushless, mLeftMaster, PDPBreaker.B40A, false);
		mLeftSlaveB.addConfigStatement((t) -> mLeftSlaveB.setCurrentLimit(CalConstants.kDriveLowGearCurrentLim));
		mLeftSlaveB.addConfigStatement((t) -> mLeftSlaveB.setClosedLoopRampRate(CalConstants.kDriveVoltageRampRate));

		mLeftSlaveB.writeToFlash();

		mRightMaster = new CKSparkMax(DeviceIDConstants.kRightDriveMasterId, CANSparkMaxLowLevel.MotorType.kBrushless, true, PDPBreaker.B40A);
		mRightMaster.addConfigStatement((t) -> mRightMaster.setInverted(true));
		mRightMaster.addConfigStatement((t) -> mRightMaster.setPIDF(CalConstants.kDriveLowGearVelocityKp, CalConstants.kDriveLowGearVelocityKi, CalConstants.kDriveLowGearVelocityKd, CalConstants.kDriveLowGearVelocityKf));
		mRightMaster.addConfigStatement((t) -> mRightMaster.setDFilter(CalConstants.kDriveLowGearVelocityDFilter));
		mRightMaster.addConfigStatement((t) -> mRightMaster.setMotionParameters(CalConstants.kDriveLowGearPositionCruiseVel, CalConstants.kDriveLowGearPositionAccel));
		mRightMaster.addConfigStatement((t) -> mRightMaster.setCurrentLimit(CalConstants.kDriveLowGearCurrentLim));
		mRightMaster.addConfigStatement((t) -> mRightMaster.setClosedLoopRampRate(CalConstants.kDriveVoltageRampRate));
		mRightMaster.setEncoderFactor(1);
		mLeftMaster.setAllowedClosedLoopError(0);
		mRightMaster.writeToFlash();

		mRightSlaveA = new CKSparkMax(DeviceIDConstants.kRightDriveSlaveAId, CANSparkMaxLowLevel.MotorType.kBrushless, mRightMaster, PDPBreaker.B40A, false);
		mRightSlaveA.addConfigStatement((t) -> mRightSlaveA.setCurrentLimit(CalConstants.kDriveLowGearCurrentLim));
		mRightSlaveA.addConfigStatement((t) -> mRightSlaveA.setClosedLoopRampRate(CalConstants.kDriveVoltageRampRate));
		mRightSlaveA.writeToFlash();

		mRightSlaveB = new CKSparkMax(DeviceIDConstants.kRightDriveSlaveBId, CANSparkMaxLowLevel.MotorType.kBrushless, mRightMaster, PDPBreaker.B40A, false);
		mRightSlaveB.addConfigStatement((t) -> mRightSlaveB.setCurrentLimit(CalConstants.kDriveLowGearCurrentLim));
		mRightSlaveB.addConfigStatement((t) -> mRightSlaveB.setClosedLoopRampRate(CalConstants.kDriveVoltageRampRate));
		mRightSlaveB.writeToFlash();

		mPTOShifter = new CKDoubleSolenoid(DeviceIDConstants.kPTOShifterSolenoidId);
		mPTOShifter.set(false);

		reloadGains();

		mGyro = new NavX();

		setOpenLoop(DriveSignal.NEUTRAL);

		// Force a CAN message across.
		mPrevBrakeMode = false;
		setBrakeMode(true);

		mMotionPlanner = new DriveMotionPlanner();

		mLeftDriveEncoderPresent = new CachedValue<>(500, (t) -> mElevator.isLeftDriveEncoderPresent());
		mRightDriveEncoderPresent = new CachedValue<>(500, (t) -> mElevator.isRightDriveEncoderPresent());
		mGyroPresent = new CachedValue<>(500, (t) -> mGyro.isPresent());

//		TuneablePIDOSC x;
//		try {
//			mLeftMaster.setControlMode(MCControlMode.Velocity);
//			mRightMaster.setControlMode(MCControlMode.Velocity);
//			x = new TuneablePIDOSC("Drive", 5804, true, mLeftMaster, mRightMaster);
//		} catch (Exception ignored) {
//
//		}
	}

	public void configureClimbCurrentLimit() {
		setBrakeMode(true);
		mLeftMaster.addConfigStatement((t) -> mLeftMaster.setSmartCurrentLimit(CalConstants.kDriveLeftClimbCurrentLim));
		mLeftSlaveA.addConfigStatement((t) -> mLeftSlaveA.setSmartCurrentLimit(CalConstants.kDriveLeftClimbCurrentLim));
		mLeftSlaveB.addConfigStatement((t) -> mLeftSlaveB.setSmartCurrentLimit(CalConstants.kDriveLeftClimbCurrentLim));

		mRightMaster.addConfigStatement((t) -> mRightMaster.setSmartCurrentLimit(CalConstants.kDriveRightClimbCurrentLim));
		mRightSlaveA.addConfigStatement((t) -> mRightSlaveA.setSmartCurrentLimit(CalConstants.kDriveRightClimbCurrentLim));
		mRightSlaveB.addConfigStatement((t) -> mRightSlaveB.setSmartCurrentLimit(CalConstants.kDriveRightClimbCurrentLim));
	}

	public void configureRetractCurrentLimit() {
		mLeftMaster.addConfigStatement((t) -> mLeftMaster.setSmartCurrentLimit(CalConstants.kDriveLeftRetractCurrentLim));
		mLeftSlaveA.addConfigStatement((t) -> mLeftSlaveA.setSmartCurrentLimit(CalConstants.kDriveLeftRetractCurrentLim));
		mLeftSlaveB.addConfigStatement((t) -> mLeftSlaveB.setSmartCurrentLimit(CalConstants.kDriveLeftRetractCurrentLim));
	}

	public static Drive getInstance() {
		return mInstance;
	}

	private static double rotationsToInches(double rotations) {
		return rotations * (CalConstants.kDriveWheelDiameterInches * Math.PI);
	}

	private static double rpmToInchesPerSecond(double rpm) {
		return rotationsToInches(rpm) / 60.0;
	}

	private static double inchesToRotations(double inches) {
		return inches / (CalConstants.kDriveWheelDiameterInches * Math.PI);
	}

	private static double inchesPerSecondToRpm(double inches_per_second) {
		return inchesToRotations(inches_per_second) * 60.0;
	}

	private static double radiansPerSecondToRPM(double rad_s) {
		return rad_s / (2.0 * Math.PI) * 60.0;
	}

	@Override
	public void registerEnabledLoops(ILooper in) {
		in.register(mLoop);
	}

	public synchronized void setOpenLoop(DriveSignal signal) {
		if (mDriveControlState != DriveControlState.OPEN_LOOP) {
			setBrakeMode(false);
			setDriveControlState(DriveControlState.OPEN_LOOP);
		}
		mPeriodicIO.left_demand = signal.getLeft();
		mPeriodicIO.right_demand = signal.getRight();
		mPeriodicIO.left_feedforward = 0.0;
		mPeriodicIO.right_feedforward = 0.0;
	}

	public synchronized void setOpenLoopAutomated(DriveSignal signal) {
		if (mDriveControlState != DriveControlState.OPEN_LOOP_AUTOMATED) {
			setBrakeMode(true);
			setDriveControlState(DriveControlState.OPEN_LOOP_AUTOMATED);
		}
		mPeriodicIO.left_demand = signal.getLeft();
		mPeriodicIO.right_demand = signal.getRight();
		mPeriodicIO.left_feedforward = 0.0;
		mPeriodicIO.right_feedforward = 0.0;
	}

	public synchronized void setClimbLeft(double leftSignal) {
		if (mDriveControlState != DriveControlState.CLIMB) {
			setBrakeMode(true);
			setDriveControlState(DriveControlState.CLIMB);
		}
		mPeriodicIO.left_demand = leftSignal;
		mPeriodicIO.left_feedforward = 0.0;
	}

	public synchronized void setClimbRight(double rightSignal) {
		if (mDriveControlState != DriveControlState.CLIMB) {
			setBrakeMode(true);
			setDriveControlState(DriveControlState.CLIMB);
		}
		mPeriodicIO.right_demand = rightSignal;
		mPeriodicIO.right_feedforward = 0.0;
	}

	public DriveControlState getDriveControlState() {
		return mDriveControlState;
	}

	public synchronized void setVelocity(DriveSignal signal, DriveSignal feedforward) {
		if (mDriveControlState != DriveControlState.PATH_FOLLOWING) {
			setBrakeMode(true);
			mLeftMaster.setPIDGainSlot(kLowGearVelocityControlSlot);
			mRightMaster.setPIDGainSlot(kLowGearVelocityControlSlot);

			setDriveControlState(DriveControlState.PATH_FOLLOWING);
		}
		mPeriodicIO.left_demand = signal.getLeft();
		mPeriodicIO.right_demand = signal.getRight();
		mPeriodicIO.left_feedforward = feedforward.getLeft();
		mPeriodicIO.right_feedforward = feedforward.getRight();
	}

	public synchronized void setTrajectory(TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
		if (mMotionPlanner != null) {
			mOverrideTrajectory = false;
			mMotionPlanner.reset();
			mMotionPlanner.setTrajectory(trajectory);
			setDriveControlState(DriveControlState.PATH_FOLLOWING);
		}
	}

	public boolean isDoneWithTrajectory() {
		if (mMotionPlanner == null || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
			return false;
		}
		return mMotionPlanner.isDone() || mOverrideTrajectory;
	}

	public boolean isHighGear() {
		return false;
	}

	public synchronized void setHighGear(boolean wantsHighGear) {

	}

	public boolean isBrakeMode() {
		return mIsBrakeMode.get();
	}

	public void setBrakeMode(boolean on) {
		mIsBrakeMode.set(on);
	}

	public synchronized void setDriveControlState(DriveControlState driveControlState) {
		mDriveControlState = driveControlState;
	}

	public double getPitch() {
		return mPeriodicIO.gyro_pitch;
	}

	public double getRoll() {
		return mPeriodicIO.gyro_roll;
	}

	public double getRawYaw() {
		return mPeriodicIO.gyro_raw_yaw;
	}

	public synchronized Rotation2d getHeading() {
		return mPeriodicIO.gyro_heading;
	}

	public synchronized void setHeading(Rotation2d heading) {
        ConsoleReporter.report("SET HEADING: " + heading.getDegrees());

        mGyroOffset = heading.rotateBy(Rotation2d.fromDegrees(mGyro.getFusedHeading()).inverse());
//        ConsoleReporter.report("Gyro offset: " + mGyroOffset.getDegrees());

        mPeriodicIO.gyro_heading = heading;
	}

	@Override
	public void stop() {
		setOpenLoop(DriveSignal.NEUTRAL);
	}

	public synchronized void resetEncoders() {
        mLeftMaster.setEncoderPosition(0);
        mRightMaster.setEncoderPosition(0);
		mElevator.zeroDriveEncoders();
		mPeriodicIO = new PeriodicIO();
	}

	@Override
	public void zeroSensors() {
		setHeading(Rotation2d.identity());
		resetEncoders();
	}

//	public double getRawLeftEncoder() {
//		return mPeriodicIO.left_position_rotations;
//	}

	public double getRawLeftSparkEncoder() {
		return mPeriodicIO.left_spark_position;
	}

	public double getLeftEncoderDistance() {
		return rotationsToInches(mPeriodicIO.left_position_rotations);
	}

	public double getRightEncoderDistance() {
		return rotationsToInches(mPeriodicIO.right_position_rotations);
	}

	public double getRightLinearVelocity() {
		return rotationsToInches(mPeriodicIO.right_velocity_RPM / 60.0);
	}

	public double getLeftLinearVelocity() {
		return rotationsToInches(mPeriodicIO.left_velocity_RPM / 60.0);
	}

	public double getLinearVelocity() {
		return (getLeftLinearVelocity() + getRightLinearVelocity()) / 2.0;
	}

	public double getAngularVelocity() {
		return (getRightLinearVelocity() - getLeftLinearVelocity()) / CalConstants.kDriveWheelTrackWidthInches;
	}

	public double getAverageInputVoltage() {
		return (mPeriodicIO.left_bus_voltage + mPeriodicIO.right_bus_voltage) / 2.0;
	}

	public void overrideTrajectory(boolean value) {
		mOverrideTrajectory = value;
	}

	public double getLeftEncoderVelocityRPM() {
		return mPeriodicIO.left_velocity_RPM;
	}

	public double getRightEncoderVelocityRPM() {
		return mPeriodicIO.right_velocity_RPM;
	}

	private void updatePathFollower() {
		if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
			final double now = Timer.getFPGATimestamp();

			DriveMotionPlanner.Output output = mMotionPlanner.update(now, RobotState.getInstance().getFieldToVehicle(now));

			mPeriodicIO.error = mMotionPlanner.error();
			mPeriodicIO.path_setpoint = mMotionPlanner.setpoint();

			if (!mOverrideTrajectory) {
				setVelocity(new DriveSignal(radiansPerSecondToRPM(output.left_velocity) * CalConstants.kDriveGearRatioMotorConversionFactor,
										   radiansPerSecondToRPM(output.right_velocity) * CalConstants.kDriveGearRatioMotorConversionFactor),
						new DriveSignal(output.left_feedforward_voltage / 12.0, output.right_feedforward_voltage / 12.0));

				mPeriodicIO.left_accel = radiansPerSecondToRPM(output.left_accel) / 1000.0;
				mPeriodicIO.right_accel = radiansPerSecondToRPM(output.right_accel) / 1000.0;
			} else {
				setVelocity(DriveSignal.BRAKE, DriveSignal.BRAKE);
				mPeriodicIO.left_accel = mPeriodicIO.right_accel = 0.0;
			}
		} else {
			ConsoleReporter.report("Drive is not in path following state", MessageLevel.ERROR);
		}
	}

	public void setPTO(boolean driveClimber) {
		mPTOShifter.set(driveClimber);
	}

	public synchronized void reloadGains() {
		mLeftMaster.setPIDF(CalConstants.kDriveLowGearVelocityKp, CalConstants.kDriveLowGearVelocityKi, CalConstants.kDriveLowGearVelocityKd, CalConstants.kDriveLowGearVelocityKf);
		mLeftMaster.setIZone(CalConstants.kDriveLowGearVelocityIZone);
		mLeftMaster.writeToFlash();

		mRightMaster.setPIDF(CalConstants.kDriveLowGearVelocityKp, CalConstants.kDriveLowGearVelocityKi, CalConstants.kDriveLowGearVelocityKd, CalConstants.kDriveLowGearVelocityKf);
		mRightMaster.setIZone(CalConstants.kDriveLowGearVelocityIZone);
		mRightMaster.writeToFlash();
	}

	@Override
	public synchronized void readPeriodicInputs() {
		double prevLeftRotations = mPeriodicIO.left_position_rotations;
		double prevRightRotations = mPeriodicIO.right_position_rotations;
		mPeriodicIO.left_position_rotations = mElevator.getLeftDrivePosition();
		mPeriodicIO.right_position_rotations = mElevator.getRightDrivePosition();
		mPeriodicIO.left_velocity_RPM = mElevator.getLeftDriveVelocity();
		mPeriodicIO.right_velocity_RPM = mElevator.getRightDriveVelocity();
		mPeriodicIO.gyro_heading = Rotation2d.fromDegrees(mGyro.getFusedHeading()).rotateBy(mGyroOffset);
		mPeriodicIO.gyro_raw_yaw = mGyro.getRawYawDegrees();
		mPeriodicIO.gyro_pitch = mGyro.getPitch();
		mPeriodicIO.gyro_roll = mGyro.getRoll();

		mPeriodicIO.left_spark_position = mLeftMaster.getPosition();
		mPeriodicIO.left_spark_velocity = mLeftMaster.getVelocity();
		mPeriodicIO.right_spark_velocity = mRightMaster.getVelocity();

		mPeriodicIO.left_drive_encoder_present = mLeftDriveEncoderPresent.getValue();
		mPeriodicIO.right_drive_encoder_present = mRightDriveEncoderPresent.getValue();
		mPeriodicIO.gyro_present = mGyroPresent.getValue();

		mPeriodicIO.left_bus_voltage = mLeftMaster.getMCInputVoltage();
		mPeriodicIO.right_bus_voltage = mRightMaster.getMCInputVoltage();

		double deltaLeftRotations = (mPeriodicIO.left_position_rotations - prevLeftRotations) * Math.PI;
		if (deltaLeftRotations > 0.0) {
			mPeriodicIO.left_distance += deltaLeftRotations * CalConstants.kDriveWheelDiameterInches;
		} else {
			mPeriodicIO.left_distance += deltaLeftRotations * CalConstants.kDriveWheelDiameterInches;
		}

		double deltaRightRotations = (mPeriodicIO.right_position_rotations - prevRightRotations) * Math.PI;
		if (deltaRightRotations > 0.0) {
			mPeriodicIO.right_distance += deltaRightRotations * CalConstants.kDriveWheelDiameterInches;
		} else {
			mPeriodicIO.right_distance += deltaRightRotations * CalConstants.kDriveWheelDiameterInches;
		}

		if (mCSVWriter != null) {
			mCSVWriter.add(mPeriodicIO);
		}

//		ConsoleReporter.report(mPeriodicIO.left_demand);
//		ConsoleReporter.report(mPeriodicIO.right_demand);
//		ConsoleReporter.report(mDriveControlState.toString());
	}

	@Override
	public synchronized void writePeriodicOutputs() {
		if (mDriveControlState == DriveControlState.OPEN_LOOP
				|| mDriveControlState == DriveControlState.CLIMB
				|| mDriveControlState == DriveControlState.OPEN_LOOP_AUTOMATED) {
			mLeftMaster.set(MCControlMode.PercentOut, mPeriodicIO.left_demand, 0, 0.0);
			mRightMaster.set(MCControlMode.PercentOut, mPeriodicIO.right_demand, 0, 0.0);
		} else if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
			mLeftMaster.set(MCControlMode.Velocity, mPeriodicIO.left_demand, 0,
					mPeriodicIO.left_feedforward + CalConstants.kDriveLowGearVelocityKd * mPeriodicIO.left_accel / mLeftMaster.getNativeUnitsOutputRange());
			mRightMaster.set(MCControlMode.Velocity, mPeriodicIO.right_demand, 0,
					mPeriodicIO.right_feedforward + CalConstants.kDriveLowGearVelocityKd * mPeriodicIO.right_accel / mRightMaster.getNativeUnitsOutputRange());
		}

		if (mIsBrakeMode.get() != mPrevBrakeMode || mForceBrakeUpdate.get()) {
			boolean newBrakeMode = mIsBrakeMode.get();
			MCNeutralMode mode = newBrakeMode ? MCNeutralMode.Brake : MCNeutralMode.Coast;
			mRightMaster.setBrakeCoastMode(mode);
			mRightSlaveA.setBrakeCoastMode(mode);
			mRightSlaveB.setBrakeCoastMode(mode);

			mLeftMaster.setBrakeCoastMode(mode);
			mLeftSlaveA.setBrakeCoastMode(mode);
			mLeftSlaveB.setBrakeCoastMode(mode);

			mPrevBrakeMode = newBrakeMode;

			if (mForceBrakeUpdate.get())
				mForceBrakeUpdate.set(false);
		}
	}

	@Override
	public boolean runDiagnostics() {
		if (TestConstants.ENABLE_DRIVE_TEST) {
			ConsoleReporter.report("Testing DRIVE---------------------------------");
			final double kLowCurrentThres = TestConstants.kDriveBaseTestLowCurrentThresh;
			final double kLowRpmThres = TestConstants.kDriveBaseTestLowRPMThresh;

			ArrayList<MotorDiagnostics> mAllMotorsDiagArr = new ArrayList<>();
			ArrayList<MotorDiagnostics> mLeftDiagArr = new ArrayList<>();
			ArrayList<MotorDiagnostics> mRightDiagArr = new ArrayList<>();
			mLeftDiagArr.add(new MotorDiagnostics("Drive Left Master", mLeftMaster));
			mLeftDiagArr.add(new MotorDiagnostics("Drive Left Slave 1", mLeftSlaveA, mLeftMaster));
			mLeftDiagArr.add(new MotorDiagnostics("Drive Left Slave 2", mLeftSlaveB, mLeftMaster));
			mRightDiagArr.add(new MotorDiagnostics("Drive Right Master", mRightMaster));
			mRightDiagArr.add(new MotorDiagnostics("Drive Right Slave 1", mRightSlaveA, mRightMaster));
			mRightDiagArr.add(new MotorDiagnostics("Drive Right Slave 2", mRightSlaveB, mRightMaster));

			mLeftSlaveA.follow(CANSparkMax.ExternalFollower.kFollowerDisabled, 0);
			mLeftSlaveB.follow(CANSparkMax.ExternalFollower.kFollowerDisabled, 0);
			mRightSlaveA.follow(CANSparkMax.ExternalFollower.kFollowerDisabled, 0);
			mRightSlaveA.setInverted(true);
			mRightSlaveB.follow(CANSparkMax.ExternalFollower.kFollowerDisabled, 0);
			mRightSlaveB.setInverted(true);

			mAllMotorsDiagArr.addAll(mLeftDiagArr);
			mAllMotorsDiagArr.addAll(mRightDiagArr);

			boolean failure = false;

			for (MotorDiagnostics mD : mAllMotorsDiagArr) {
				mD.setZero();
			}

			for (MotorDiagnostics mD : mAllMotorsDiagArr) {
				ConsoleReporter.report("Testing motor: " + mD.getMotorName());
				mD.runTest();

				if (mD.isCurrentUnderThreshold(kLowCurrentThres)) {
					ConsoleReporter.report("!!!!!!!!!!!!!!!!!! " + mD.getMotorName() + " Current Low !!!!!!!!!!");
					failure = true;
				}

				if (mD.isRPMUnderThreshold(kLowRpmThres)) {
					ConsoleReporter.report("!!!!!!!!!!!!!!!!!! " + mD.getMotorName() + " RPM Low !!!!!!!!!!");
					failure = true;
				}

				if (!mD.isSensorInPhase()) {
					ConsoleReporter.report("!!!!!!!!!!!!!!!!!! " + mD.getMotorName() + " Sensor Out of Phase !!!!!!!!!!");
					failure = true;
				}

				try {
					Thread.sleep(2000);
				} catch (Exception ex) {

				}
			}

			if (mLeftDiagArr.size() > 0 && mRightDiagArr.size() > 0 && mAllMotorsDiagArr.size() > 0) {
				List<Double> leftMotorCurrents = mLeftDiagArr.stream().map(MotorDiagnostics::getMotorCurrent).collect(Collectors.toList());
				if (!Util.allCloseTo(leftMotorCurrents, leftMotorCurrents.get(0), TestConstants.kDriveBaseTestCurrentDelta)) {
					failure = true;
					ConsoleReporter.report("!!!!!!!!!!!!!!!!!! Drive Left2Cube Currents Different !!!!!!!!!!");
				}

				List<Double> rightMotorCurrents = mRightDiagArr.stream().map(MotorDiagnostics::getMotorCurrent).collect(Collectors.toList());
				if (!Util.allCloseTo(rightMotorCurrents, rightMotorCurrents.get(0), TestConstants.kDriveBaseTestCurrentDelta)) {
					failure = true;
					ConsoleReporter.report("!!!!!!!!!!!!!!!!!! Drive Right2Cube Currents Different !!!!!!!!!!");
				}

				List<Double> driveMotorRPMs = mAllMotorsDiagArr.stream().map(MotorDiagnostics::getMotorRPM).collect(Collectors.toList());
				if (!Util.allCloseTo(driveMotorRPMs, driveMotorRPMs.get(0), TestConstants.kDriveBaseTestRPMDelta)) {
					failure = true;
					ConsoleReporter.report("!!!!!!!!!!!!!!!!!!! Drive RPMs different !!!!!!!!!!!!!!!!!!!");
				}
			} else {
				ConsoleReporter.report("Drive Testing Error Occurred in system. Please check code!", MessageLevel.ERROR);
			}

			return !failure;
		}
		else
			return true;
	}

	public void forceBrakeModeUpdate() {
		mForceBrakeUpdate.set(true);
	}

	@Override
	public synchronized String generateReport() {
		return mLogDataGenerator.generateData(mPeriodicIO);

//		return  "LeftDrivePos:" +mPeriodicIO.left_position_rotations + ";" +
//				"LeftDriveVel:" + mPeriodicIO.left_spark_velocity + ";" +
//				"LeftDriveOutput:" + mPeriodicIO.left_demand + ";" +
//				"LeftDrive1Current:" + mLeftMaster.getMCOutputCurrent() + ";" +
//				"LeftDrive2Current:" + mLeftSlaveA.getMCOutputCurrent() + ";" +
//				"LeftDrive3Current:" + mLeftSlaveB.getMCOutputCurrent() + ";" +
//				"LeftDrive1HasReset:" + mLeftMaster.hasMotorControllerReset() + ";" +
//				"LeftDrive2HasReset:" + mLeftSlaveA.hasMotorControllerReset() + ";" +
//				"LeftDrive3HasReset:" + mLeftSlaveB.hasMotorControllerReset() + ";" +
//				"LeftDriveOutputDutyCycle:" + mLeftMaster.getMCOutputPercent() + ";" +
//				"LeftDriveOutputVoltage:" + mLeftMaster.getMCOutputPercent() * mLeftMaster.getMCInputVoltage() + ";" +
//				"LeftDriveSupplyVoltage:" + mLeftMaster.getMCInputVoltage() + ";" +
//				"LeftDriveVelocityError:" + (mPeriodicIO.left_demand - mPeriodicIO.left_spark_velocity) + ";" +
//				"RightDrivePos:" + mPeriodicIO.right_position_rotations + ";" +
//				"RightDriveVel:" + mPeriodicIO.right_spark_velocity + ";" +
//				"RightDriveOutput:" + mPeriodicIO.right_demand + ";" +
//				"RightDrive1Current:" + mRightMaster.getMCOutputCurrent() + ";" +
//				"RightDrive2Current:" + mRightSlaveA.getMCOutputCurrent() + ";" +
//				"RightDrive3Current:" + mRightSlaveB.getMCOutputCurrent() + ";" +
//				"RightDrive1HasReset:" + mRightMaster.hasMotorControllerReset() + ";" +
//				"RightDrive2HasReset:" + mRightSlaveA.hasMotorControllerReset() + ";" +
//				"RightDrive3HasReset:" + mRightSlaveB.hasMotorControllerReset() + ";" +
//				"RightDriveOutputDutyCycle:" + mRightMaster.getMCOutputPercent() + ";" +
//				"RightDriveOutputVoltage:" + mRightMaster.getMCOutputPercent() * mRightMaster.getMCInputVoltage() + ";" +
//				"RightDriveSupplyVoltage:" + mRightMaster.getMCInputVoltage() + ";" +
//				"RightDriveVelocityError:" + (mPeriodicIO.right_demand - mPeriodicIO.right_spark_velocity) + ";" +
//				"DriveMode:" + mDriveControlState.toString() + ";" +
//				"DriveErrorX:" + mPeriodicIO.error.getTranslation().x() + ";" +
//				"DriveErrorY:" + mPeriodicIO.error.getTranslation().y() + ";" +
//				"DriveErrorRotation:" + mPeriodicIO.error.getRotation().getDegrees() + ";" +
//				"Timestamp:" + Timer.getFPGATimestamp() + ";" +
//				"IsDriveFaulted:" + isSystemFaulted() + ";";
	}

	@Override
	public synchronized boolean isSystemFaulted() {
		boolean leftSensorFaulted = !mPeriodicIO.left_drive_encoder_present;
		boolean rightSensorFaulted = !mPeriodicIO.right_drive_encoder_present;
		boolean navXFaulted = !mPeriodicIO.gyro_present;

		if (leftSensorFaulted)
			ConsoleReporter.report("Left Drive Encoder Error", MessageLevel.DEFCON1);

		if (rightSensorFaulted)
			ConsoleReporter.report("Right Drive Encoder Error", MessageLevel.DEFCON1);

		if (navXFaulted)
			ConsoleReporter.report("NavX Error", MessageLevel.DEFCON1);

		//These functions are called in the LogDataReporter, so don't call them twice to save CPU.
		//If LogDataReporter not enabled, call them here
//		mLeftMaster.hasMotorControllerReset();
//		mLeftSlaveA.hasMotorControllerReset();
//		mLeftSlaveB.hasMotorControllerReset();
//		mRightMaster.hasMotorControllerReset();
//		mRightSlaveA.hasMotorControllerReset();
//		mRightSlaveB.hasMotorControllerReset();

		return leftSensorFaulted || rightSensorFaulted || navXFaulted;
	}

	public enum DriveControlState {
		OPEN_LOOP,
		PATH_FOLLOWING,
		VELOCITY,
		CLIMB,
		OPEN_LOOP_AUTOMATED
	}

	public enum ShifterState {
		FORCE_LOW_GEAR,
		FORCE_HIGH_GEAR,
		AUTO_SHIFT
	}

	public static class PeriodicIO {
		//Making members public here will automatically add them to logs
		// INPUTS
		public double left_position_rotations;
		public double right_position_rotations;
		public double left_distance;
		public double right_distance;
		public double left_velocity_RPM;
		public double right_velocity_RPM;
		public Rotation2d gyro_heading = Rotation2d.identity();
		double gyro_raw_yaw;
		double gyro_pitch;
		public double gyro_roll;
		public Pose2d error = Pose2d.identity();

		public double left_spark_position;
		public double left_spark_velocity;
		public double right_spark_velocity;
		public double left_bus_voltage;
		public double right_bus_voltage;

		boolean left_drive_encoder_present;
		boolean right_drive_encoder_present;
		boolean gyro_present;

		// OUTPUTS
		public double left_demand;
		public double right_demand;
		public double left_accel;
		public double right_accel;
		public double left_feedforward;
		public double right_feedforward;
		TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<>(Pose2dWithCurvature.identity());
	}
}

