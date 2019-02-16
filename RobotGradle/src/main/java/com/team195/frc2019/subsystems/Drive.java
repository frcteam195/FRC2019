package com.team195.frc2019.subsystems;

import com.revrobotics.CANSparkMaxLowLevel;
import com.team195.frc2019.loops.ILooper;
import com.team195.frc2019.loops.Loop;
import com.team195.frc2019.planners.DriveMotionPlanner;
import com.team195.frc2019.Constants;
import com.team195.frc2019.RobotState;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.reporters.MessageLevel;
import com.team195.lib.drivers.CKDoubleSolenoid;
import com.team195.lib.drivers.CKIMU;
import com.team195.lib.drivers.NavX;
import com.team195.lib.drivers.motorcontrol.CKSparkMax;
import com.team195.lib.drivers.motorcontrol.MCControlMode;
import com.team195.lib.drivers.motorcontrol.MCNeutralMode;
import com.team195.lib.drivers.motorcontrol.PDPBreaker;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.ReflectingCSVWriter;
import edu.wpi.first.wpilibj.Timer;

public class Drive extends Subsystem {

	private static final int kLowGearVelocityControlSlot = 0;
	private static Drive mInstance = new Drive();
	private final CKSparkMax mLeftMaster, mRightMaster, mLeftSlaveA, mRightSlaveA, mLeftSlaveB, mRightSlaveB;
	private final CKDoubleSolenoid mPTOShifter;
	private DriveControlState mDriveControlState;
	private CKIMU mGyro;
	private PeriodicIO mPeriodicIO;
	private boolean mIsBrakeMode;
	private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;
	private DriveMotionPlanner mMotionPlanner;
	private Rotation2d mGyroOffset = Rotation2d.identity();
	private boolean mOverrideTrajectory = false;

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
				setBrakeMode(false);
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
	};

	private Drive() {
		mPeriodicIO = new PeriodicIO();

		mLeftMaster = new CKSparkMax(Constants.kLeftDriveMasterId, CANSparkMaxLowLevel.MotorType.kBrushless, true, PDPBreaker.B40A);
		mLeftMaster.setInverted(false);
		mLeftMaster.setPIDF(0.00016, 0, 0.0004, 0.000156);
		mLeftMaster.setMotionParameters(10000, 500);
		mLeftMaster.writeToFlash();

		mLeftSlaveA = new CKSparkMax(Constants.kLeftDriveSlaveAId, CANSparkMaxLowLevel.MotorType.kBrushless, mLeftMaster, PDPBreaker.B40A);
		mLeftSlaveA.setInverted(false);
		mLeftSlaveA.writeToFlash();

		mLeftSlaveB = new CKSparkMax(Constants.kLeftDriveSlaveBId, CANSparkMaxLowLevel.MotorType.kBrushless, mLeftMaster, PDPBreaker.B40A);
		mLeftSlaveB.setInverted(false);
		mLeftSlaveB.writeToFlash();

		mRightMaster = new CKSparkMax(Constants.kRightDriveMasterId, CANSparkMaxLowLevel.MotorType.kBrushless, true, PDPBreaker.B40A);
		mRightMaster.setInverted(true);
		mRightMaster.setPIDF(0.00016, 0, 0.0004, 0.000156);
		mRightMaster.setMotionParameters(10000, 500);
		mRightMaster.writeToFlash();

		mRightSlaveA = new CKSparkMax(Constants.kRightDriveSlaveAId, CANSparkMaxLowLevel.MotorType.kBrushless, mRightMaster, PDPBreaker.B40A);
		mRightSlaveA.setInverted(true);
		mRightSlaveA.writeToFlash();

		mRightSlaveB = new CKSparkMax(Constants.kRightDriveSlaveBId, CANSparkMaxLowLevel.MotorType.kBrushless, mRightMaster, PDPBreaker.B40A);
		mRightSlaveB.setInverted(true);
		mRightSlaveB.writeToFlash();

		mPTOShifter = new CKDoubleSolenoid(Constants.kPTOShifterSolenoidId);
		mPTOShifter.configReversed(false);
		mPTOShifter.set(false);

		reloadGains();

		mGyro = new NavX();

		setOpenLoop(DriveSignal.NEUTRAL);

		// Force a CAN message across.
		mIsBrakeMode = true;
		setBrakeMode(false);

		mMotionPlanner = new DriveMotionPlanner();
	}

	public static Drive getInstance() {
		return mInstance;
	}

	private static double rotationsToInches(double rotations) {
		return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
	}

	private static double rpmToInchesPerSecond(double rpm) {
		return rotationsToInches(rpm) / 60.0;
	}

	private static double inchesToRotations(double inches) {
		return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
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

			mDriveControlState = DriveControlState.OPEN_LOOP;
		}
		mPeriodicIO.left_demand = signal.getLeft();
		mPeriodicIO.right_demand = signal.getRight();
		mPeriodicIO.left_feedforward = 0.0;
		mPeriodicIO.right_feedforward = 0.0;
	}

	public synchronized void setVelocity(DriveSignal signal, DriveSignal feedforward) {
		if (mDriveControlState != DriveControlState.PATH_FOLLOWING) {
			setBrakeMode(true);
			mLeftMaster.setPIDGainSlot(kLowGearVelocityControlSlot);
			mRightMaster.setPIDGainSlot(kLowGearVelocityControlSlot);

			mDriveControlState = DriveControlState.PATH_FOLLOWING;
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
			mDriveControlState = DriveControlState.PATH_FOLLOWING;
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
		return mIsBrakeMode;
	}

	public synchronized void setBrakeMode(boolean on) {
		if (mIsBrakeMode != on) {
			mIsBrakeMode = on;
			MCNeutralMode mode = on ? MCNeutralMode.Brake : MCNeutralMode.Coast;
			mRightMaster.setBrakeCoastMode(mode);
			mRightSlaveA.setBrakeCoastMode(mode);
			mRightSlaveB.setBrakeCoastMode(mode);

			mLeftMaster.setBrakeCoastMode(mode);
			mLeftSlaveA.setBrakeCoastMode(mode);
			mLeftSlaveB.setBrakeCoastMode(mode);
		}
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
	public synchronized void stop() {
		setOpenLoop(DriveSignal.NEUTRAL);
	}

//	@Override
//	public void outputTelemetry() {
//		SmartDashboard.putNumber("Right Drive Distance", mPeriodicIO.right_distance);
//		SmartDashboard.putNumber("Right Drive Ticks", mPeriodicIO.right_position_rotations);
//		SmartDashboard.putNumber("Left Drive Ticks", mPeriodicIO.left_position_rotations);
//		SmartDashboard.putNumber("Left Drive Distance", mPeriodicIO.left_distance);
//		SmartDashboard.putNumber("Right Linear Velocity", getRightLinearVelocity());
//		SmartDashboard.putNumber("Left Linear Velocity", getLeftLinearVelocity());
//
//		SmartDashboard.putNumber("x err", mPeriodicIO.error.getTranslation().x());
//		SmartDashboard.putNumber("y err", mPeriodicIO.error.getTranslation().y());
//		SmartDashboard.putNumber("theta err", mPeriodicIO.error.getRotation().getDegrees());
//		if (getHeading() != null) {
//			SmartDashboard.putNumber("Gyro Heading", getHeading().getDegrees());
//		}
//		if (mCSVWriter != null) {
//			mCSVWriter.write();
//		}
//	}

	public synchronized void resetEncoders() {
        mLeftMaster.setEncoderPosition(0);
        mRightMaster.setEncoderPosition(0);
		mPeriodicIO = new PeriodicIO();
	}

	@Override
	public void zeroSensors() {
		setHeading(Rotation2d.identity());
		resetEncoders();
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
		return (getRightLinearVelocity() - getLeftLinearVelocity()) / Constants.kDriveWheelTrackWidthInches;
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
				setVelocity(new DriveSignal(radiansPerSecondToRPM(output.left_velocity), radiansPerSecondToRPM(output.right_velocity)),
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
		mLeftMaster.setPIDF(Constants.kDriveLowGearVelocityKp, Constants.kDriveLowGearVelocityKi, Constants.kDriveLowGearVelocityKd, Constants.kDriveLowGearVelocityKf);
		mLeftMaster.setIZone(Constants.kDriveLowGearVelocityIZone);
		mLeftMaster.writeToFlash();

		mRightMaster.setPIDF(Constants.kDriveLowGearVelocityKp, Constants.kDriveLowGearVelocityKi, Constants.kDriveLowGearVelocityKd, Constants.kDriveLowGearVelocityKf);
		mRightMaster.setIZone(Constants.kDriveLowGearVelocityIZone);
		mRightMaster.writeToFlash();
	}

	@Override
	public synchronized void readPeriodicInputs() {
		double prevLeftRotations = mPeriodicIO.left_position_rotations;
		double prevRightRotations = mPeriodicIO.right_position_rotations;
		mPeriodicIO.left_position_rotations = mLeftMaster.getPosition();
		mPeriodicIO.right_position_rotations = mRightMaster.getPosition();
		mPeriodicIO.left_velocity_RPM = mLeftMaster.getVelocity();
		mPeriodicIO.right_velocity_RPM = mRightMaster.getVelocity();
		mPeriodicIO.gyro_heading = Rotation2d.fromDegrees(mGyro.getFusedHeading()).rotateBy(mGyroOffset);

		double deltaLeftRotations = (mPeriodicIO.left_position_rotations - prevLeftRotations) * Math.PI;
		if (deltaLeftRotations > 0.0) {
			mPeriodicIO.left_distance += deltaLeftRotations * Constants.kDriveWheelDiameterInches;
		} else {
			mPeriodicIO.left_distance += deltaLeftRotations * Constants.kDriveWheelDiameterInches;
		}

		double deltaRightRotations = (mPeriodicIO.right_position_rotations - prevRightRotations) * Math.PI;
		if (deltaRightRotations > 0.0) {
			mPeriodicIO.right_distance += deltaRightRotations * Constants.kDriveWheelDiameterInches;
		} else {
			mPeriodicIO.right_distance += deltaRightRotations * Constants.kDriveWheelDiameterInches;
		}

		if (mCSVWriter != null) {
			mCSVWriter.add(mPeriodicIO);
		}
	}

	@Override
	public synchronized void writePeriodicOutputs() {
		if (mDriveControlState == DriveControlState.OPEN_LOOP) {
			mLeftMaster.set(MCControlMode.PercentOut, mPeriodicIO.left_demand, 0, 0.0);
			mRightMaster.set(MCControlMode.PercentOut, mPeriodicIO.right_demand, 0, 0.0);
		} else {
			mLeftMaster.set(MCControlMode.Velocity, mPeriodicIO.left_demand, 0,
					mPeriodicIO.left_feedforward + Constants.kDriveLowGearVelocityKd * mPeriodicIO.left_accel / mLeftMaster.getNativeUnitsOutputRange());
			mRightMaster.set(MCControlMode.Velocity, mPeriodicIO.right_demand, 0,
					mPeriodicIO.right_feedforward + Constants.kDriveLowGearVelocityKd * mPeriodicIO.right_accel / mRightMaster.getNativeUnitsOutputRange());
		}
	}

	@Override
	public boolean runDiagnostics() {
//        boolean leftSide = TalonSRXChecker.CheckTalons(this,
//                new ArrayList<TalonSRXChecker.TalonSRXConfig>() {
//                    {
//                        add(new TalonSRXChecker.TalonSRXConfig("left_master", mLeftMaster));
//                        add(new TalonSRXChecker.TalonSRXConfig("left_slave", mLeftSlaveA));
//                        add(new TalonSRXChecker.TalonSRXConfig("left_slave1", mLeftSlaveB));
//                    }
//                }, new TalonSRXChecker.CheckerConfig() {
//                    {
//                        mCurrentFloor = 2;
//                        mRPMFloor = 1500;
//                        mCurrentEpsilon = 2.0;
//                        mRPMEpsilon = 250;
//                        mRPMSupplier = () -> mLeftMaster.getSelectedSensorVelocity(0);
//                    }
//                });
//        boolean rightSide = TalonSRXChecker.CheckTalons(this,
//                new ArrayList<TalonSRXChecker.TalonSRXConfig>() {
//                    {
//                        add(new TalonSRXChecker.TalonSRXConfig("right_master", mRightMaster));
//                        add(new TalonSRXChecker.TalonSRXConfig("right_slave", mRightSlaveA));
//                        add(new TalonSRXChecker.TalonSRXConfig("right_slave1", mRightSlaveB));
//                    }
//                }, new TalonSRXChecker.CheckerConfig() {
//                    {
//                        mCurrentFloor = 2;
//                        mRPMFloor = 1500;
//                        mCurrentEpsilon = 2.0;
//                        mRPMEpsilon = 250;
//                        mRPMSupplier = () -> mRightMaster.getSelectedSensorVelocity(0);
//                    }
//                });
//        return leftSide && rightSide;
		return false;
	}

	@Override
	public String generateReport() {
		String retVal = "";

		retVal += "LeftDrivePos:" + mLeftMaster.getVelocity() + ";";
		retVal += "LeftDriveVel:" + mLeftMaster.getVelocity() + ";";
		retVal += "LeftDriveOutput:" + mPeriodicIO.left_demand + ";";
		retVal += "LeftDrive1Current:" + mLeftMaster.getMCOutputCurrent() + ";";
		retVal += "LeftDrive2Current:" + mLeftSlaveA.getMCOutputCurrent() + ";";
		retVal += "LeftDrive3Current:" + mLeftSlaveB.getMCOutputCurrent() + ";";
		retVal += "LeftDriveOutputDutyCycle:" + mLeftMaster.getMCOutputPercent() + ";";
		retVal += "LeftDriveOutputVoltage:" + mLeftMaster.getMCOutputPercent()*mLeftMaster.getMCInputVoltage() + ";";
		retVal += "LeftDriveSupplyVoltage:" + mLeftMaster.getMCInputVoltage() + ";";

		retVal += "RightDrivePos:" + mRightMaster.getVelocity() + ";";
		retVal += "RightDriveVel:" + mRightMaster.getVelocity() + ";";
		retVal += "RightDriveOutput:" + mPeriodicIO.right_demand + ";";
		retVal += "RightDrive1Current:" + mRightMaster.getMCOutputCurrent() + ";";
		retVal += "RightDrive2Current:" + mRightSlaveA.getMCOutputCurrent() + ";";
		retVal += "RightDrive3Current:" + mRightSlaveB.getMCOutputCurrent() + ";";
		retVal += "RightDriveOutputDutyCycle:" + mRightMaster.getMCOutputPercent() + ";";
		retVal += "RightDriveOutputVoltage:" + mRightMaster.getMCOutputPercent()*mRightMaster.getMCInputVoltage() + ";";
		retVal += "RightDriveSupplyVoltage:" + mRightMaster.getBusVoltage() + ";";

//		retVal += "AccelX:" + mGyro.getRawAccelX() + ";";
//		retVal += "AccelY:" + mGyro.getRawAccelY() + ";";
//		retVal += "AccelZ:" + mGyro.getRawAccelZ() + ";";
//
//		retVal += "Gyro:" + mGyro.getRawYawDegrees() + ";";
//		retVal += "GyroRate:" + mGyro.getYawRateDegreesPerSec() + ";";

		retVal += "DriveMode:" + mDriveControlState.toString() + ";";
//		retVal += "RobotPosition:" + PathFollowerRobotState.getInstance().getLatestFieldToVehicle().getValue().toString() + ";";

		retVal += "IsDriveFaulted:" + isSystemFaulted() + ";";

		return retVal;
	}

	@Override
	public boolean isSystemFaulted() {
		boolean leftSensorFaulted = !mLeftMaster.isEncoderPresent();
		boolean rightSensorFaulted = !mRightMaster.isEncoderPresent();
		boolean navXFaulted = !mGyro.isPresent();

		if (leftSensorFaulted)
			ConsoleReporter.report("Left Drive Encoder Error", MessageLevel.DEFCON1);

		if (rightSensorFaulted)
			ConsoleReporter.report("Right Drive Encoder Error", MessageLevel.DEFCON1);

		if (navXFaulted)
			ConsoleReporter.report("NavX Error", MessageLevel.DEFCON1);

		return leftSensorFaulted || rightSensorFaulted || navXFaulted;
	}

	public enum DriveControlState {
		OPEN_LOOP,
		PATH_FOLLOWING,
	}

	public enum ShifterState {
		FORCE_LOW_GEAR,
		FORCE_HIGH_GEAR,
		AUTO_SHIFT
	}

	public static class PeriodicIO {
		// INPUTS
		public double left_position_rotations;
		public double right_position_rotations;
		public double left_distance;
		public double right_distance;
		public double left_velocity_RPM;
		public double right_velocity_RPM;
		public Rotation2d gyro_heading = Rotation2d.identity();
		public Pose2d error = Pose2d.identity();

		// OUTPUTS
		public double left_demand;
		public double right_demand;
		public double left_accel;
		public double right_accel;
		public double left_feedforward;
		public double right_feedforward;
		public TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<Pose2dWithCurvature>(Pose2dWithCurvature.identity());
	}
}

