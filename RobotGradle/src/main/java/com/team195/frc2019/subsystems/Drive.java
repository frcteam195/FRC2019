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
import com.team195.lib.drivers.motorcontrol.*;
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
import java.util.stream.Collectors;

public class Drive extends Subsystem {

	private static final int kLowGearVelocityControlSlot = 0;
	private static Drive mInstance = new Drive();
	private final CKSparkMax mLeftMaster, mRightMaster, mLeftSlaveA, mRightSlaveA, mLeftSlaveB, mRightSlaveB;
	private final CKDoubleSolenoid mPTOShifter;
	private DriveControlState mDriveControlState;
	private BrakeState mBrakeState = BrakeState.MOTOR_MASTER;
	private BrakeState mPrevBrakeState = BrakeState.MOTOR_SLAVEB;
	private CKIMU mGyro;
	private PeriodicIO mPeriodicIO;
	private boolean mIsBrakeMode;
	private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;
	private DriveMotionPlanner mMotionPlanner;
	private Rotation2d mGyroOffset = Rotation2d.identity();
	private boolean mOverrideTrajectory = false;
	private double mLastBrakeSwitch = Timer.getFPGATimestamp();
	private boolean mBrakeSwitchEnabled = true;

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
					setBrake(mLeftMaster, mRightMaster);
					setCoast(mLeftSlaveA, mLeftSlaveB, mRightSlaveA, mRightSlaveB);
				}
				else {
					setBrakeMode(false);
					setBrakeMode(true);
				}
			}
		}

		@Override
		public void onLoop(double timestamp) {
			synchronized (Drive.this) {
				switch (mDriveControlState) {
					case OPEN_LOOP:
						if (mBrakeSwitchEnabled) {
							if ((Timer.getFPGATimestamp() - mLastBrakeSwitch) > 30) {
								mLastBrakeSwitch = Timer.getFPGATimestamp();

								switch (mBrakeState) {
									case MOTOR_MASTER:
										mBrakeState = BrakeState.MOTOR_SLAVEA;
										break;
									case MOTOR_SLAVEA:
										mBrakeState = BrakeState.MOTOR_SLAVEB;
										break;
									case MOTOR_SLAVEB:
										mBrakeState = BrakeState.MOTOR_MASTER;
										break;
								}
							}
						}
						break;
					case PATH_FOLLOWING:
						updatePathFollower();
						break;
					case CLIMB:
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

	private void setBrake(CKSparkMax... brakeMCs) {
		if (brakeMCs != null && brakeMCs.length > 0) {
			for (CKSparkMax cksm : brakeMCs) {
				cksm.setBrakeCoastMode(MCNeutralMode.Brake);
			}
		}
	}

	private void setCoast(CKSparkMax... coastMCs) {
		if (coastMCs != null && coastMCs.length > 0) {
			for (CKSparkMax cksm : coastMCs) {
				cksm.setBrakeCoastMode(MCNeutralMode.Coast);
			}
		}
	}

	private Drive() {
		mPeriodicIO = new PeriodicIO();

		mLeftMaster = new CKSparkMax(Constants.kLeftDriveMasterId, CANSparkMaxLowLevel.MotorType.kBrushless, true, PDPBreaker.B40A);
		mLeftMaster.addConfigStatement((t) -> mLeftMaster.setInverted(false));
		mLeftMaster.addConfigStatement((t) -> mLeftMaster.setPIDF(Constants.kDriveLowGearPositionKp, Constants.kDriveLowGearPositionKi, Constants.kDriveLowGearPositionKd, Constants.kDriveLowGearPositionKf));
		mLeftMaster.addConfigStatement((t) -> mLeftMaster.setDFilter(Constants.kDriveLowGearPositionDFilter));
		mLeftMaster.addConfigStatement((t) -> mLeftMaster.setMotionParameters(Constants.kDriveLowGearPositionCruiseVel, Constants.kDriveLowGearPositionAccel));
		mLeftMaster.addConfigStatement((t) -> mLeftMaster.setSmartCurrentLimit(Constants.kDriveLowGearCurrentLim));
		mLeftMaster.addConfigStatement((t) -> mLeftMaster.writeToFlash());

		mLeftSlaveA = new CKSparkMax(Constants.kLeftDriveSlaveAId, CANSparkMaxLowLevel.MotorType.kBrushless, mLeftMaster, PDPBreaker.B40A, false);
		mLeftSlaveA.addConfigStatement((t) -> mLeftSlaveA.setSmartCurrentLimit(Constants.kDriveLowGearCurrentLim));
		mLeftSlaveA.addConfigStatement((t) -> mLeftSlaveA.writeToFlash());

		mLeftSlaveB = new CKSparkMax(Constants.kLeftDriveSlaveBId, CANSparkMaxLowLevel.MotorType.kBrushless, mLeftMaster, PDPBreaker.B40A, false);
		mLeftSlaveB.addConfigStatement((t) -> mLeftSlaveB.setSmartCurrentLimit(Constants.kDriveLowGearCurrentLim));
		mLeftSlaveB.addConfigStatement((t) -> mLeftSlaveB.writeToFlash());

		mRightMaster = new CKSparkMax(Constants.kRightDriveMasterId, CANSparkMaxLowLevel.MotorType.kBrushless, true, PDPBreaker.B40A);
		mRightMaster.addConfigStatement((t) -> mRightMaster.setInverted(true));
		mRightMaster.addConfigStatement((t) -> mRightMaster.setPIDF(Constants.kDriveLowGearPositionKp, Constants.kDriveLowGearPositionKi, Constants.kDriveLowGearPositionKd, Constants.kDriveLowGearPositionKf));
		mRightMaster.addConfigStatement((t) -> mRightMaster.setDFilter(Constants.kDriveLowGearPositionDFilter));
		mRightMaster.addConfigStatement((t) -> mRightMaster.setMotionParameters(Constants.kDriveLowGearPositionCruiseVel, Constants.kDriveLowGearPositionAccel));
		mRightMaster.addConfigStatement((t) -> mRightMaster.setSmartCurrentLimit(Constants.kDriveLowGearCurrentLim));
		mRightMaster.addConfigStatement((t) -> mRightMaster.writeToFlash());

		mRightSlaveA = new CKSparkMax(Constants.kRightDriveSlaveAId, CANSparkMaxLowLevel.MotorType.kBrushless, mRightMaster, PDPBreaker.B40A, false);
		mRightSlaveA.addConfigStatement((t) -> mRightSlaveA.setSmartCurrentLimit(Constants.kDriveLowGearCurrentLim));
		mRightSlaveA.addConfigStatement((t) -> mRightSlaveA.writeToFlash());

		mRightSlaveB = new CKSparkMax(Constants.kRightDriveSlaveBId, CANSparkMaxLowLevel.MotorType.kBrushless, mRightMaster, PDPBreaker.B40A, false);
		mRightSlaveB.addConfigStatement((t) -> mRightSlaveB.setSmartCurrentLimit(Constants.kDriveLowGearCurrentLim));
		mRightSlaveB.addConfigStatement((t) -> mRightSlaveB.writeToFlash());

		mPTOShifter = new CKDoubleSolenoid(Constants.kPTOShifterSolenoidId);
		mPTOShifter.set(false);

		reloadGains();

		mGyro = new NavX();

		setOpenLoop(DriveSignal.NEUTRAL);

		// Force a CAN message across.
		mIsBrakeMode = true;
		setBrakeMode(false);


		mMotionPlanner = new DriveMotionPlanner();
	}

	public void configureClimbCurrentLimit() {
		mBrakeSwitchEnabled = false;
		setBrakeMode(true);
		mLeftMaster.addConfigStatement((t) -> mLeftMaster.setSmartCurrentLimit(Constants.kDriveLeftClimbCurrentLim));
		mLeftSlaveA.addConfigStatement((t) -> mLeftSlaveA.setSmartCurrentLimit(Constants.kDriveLeftClimbCurrentLim));
		mLeftSlaveB.addConfigStatement((t) -> mLeftSlaveB.setSmartCurrentLimit(Constants.kDriveLeftClimbCurrentLim));

		mRightMaster.addConfigStatement((t) -> mRightMaster.setSmartCurrentLimit(Constants.kDriveRightClimbCurrentLim));
		mRightSlaveA.addConfigStatement((t) -> mRightSlaveA.setSmartCurrentLimit(Constants.kDriveRightClimbCurrentLim));
		mRightSlaveB.addConfigStatement((t) -> mRightSlaveB.setSmartCurrentLimit(Constants.kDriveRightClimbCurrentLim));
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

	public synchronized void setBobbyBrake() {
		if (mBrakeSwitchEnabled && mBrakeState != mPrevBrakeState) {
			setInternalBrakeMode(false);
			switch (mBrakeState) {
				case MOTOR_MASTER:
					setBrake(mLeftMaster, mRightMaster);
					setCoast(mLeftSlaveA, mLeftSlaveB, mRightSlaveA, mRightSlaveB);
					break;
				case MOTOR_SLAVEA:
					setBrake(mLeftSlaveA, mRightSlaveA);
					setCoast(mLeftMaster, mLeftSlaveB, mRightMaster, mRightSlaveB);
					break;
				case MOTOR_SLAVEB:
					setBrake(mLeftSlaveB, mRightSlaveB);
					setCoast(mLeftMaster, mLeftSlaveA, mRightMaster, mRightSlaveA);
					break;
			}
			mPrevBrakeState = mBrakeState;
		}
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

	public synchronized void setClimbLeft(double leftPos) {
		if (mDriveControlState != DriveControlState.CLIMB) {
			setBrakeMode(true);

			setDriveControlState(DriveControlState.CLIMB);
		}
		mPeriodicIO.left_demand = leftPos;
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
		return mIsBrakeMode;
	}

	public synchronized void setBrakeMode(boolean on) {
		if (mIsBrakeMode != on) {
			setInternalBrakeMode(on);
			MCNeutralMode mode = on ? MCNeutralMode.Brake : MCNeutralMode.Coast;
			mRightMaster.setBrakeCoastMode(mode);
			mRightSlaveA.setBrakeCoastMode(mode);
			mRightSlaveB.setBrakeCoastMode(mode);

			mLeftMaster.setBrakeCoastMode(mode);
			mLeftSlaveA.setBrakeCoastMode(mode);
			mLeftSlaveB.setBrakeCoastMode(mode);
		}
	}

	private synchronized void setInternalBrakeMode(boolean on) {
		mIsBrakeMode = on;
	}

	public synchronized void setDriveControlState(DriveControlState driveControlState) {
		mDriveControlState = driveControlState;
	}

	public synchronized double getPitch() {
		return mPeriodicIO.gyro_pitch;
	}

	public synchronized double getRoll() {
		return mPeriodicIO.gyro_roll;
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
		mPeriodicIO.gyro_pitch = mGyro.getPitch();
		mPeriodicIO.gyro_roll = mGyro.getRoll();

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
		} else if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
			mLeftMaster.set(MCControlMode.Velocity, mPeriodicIO.left_demand, 0,
					mPeriodicIO.left_feedforward + Constants.kDriveLowGearVelocityKd * mPeriodicIO.left_accel / mLeftMaster.getNativeUnitsOutputRange());
			mRightMaster.set(MCControlMode.Velocity, mPeriodicIO.right_demand, 0,
					mPeriodicIO.right_feedforward + Constants.kDriveLowGearVelocityKd * mPeriodicIO.right_accel / mRightMaster.getNativeUnitsOutputRange());
		} else if (mDriveControlState == DriveControlState.CLIMB) {
			mLeftMaster.set(MCControlMode.PercentOut, mPeriodicIO.left_demand, 0, 0.0);
			mRightMaster.set(MCControlMode.PercentOut, mPeriodicIO.right_demand, 0, 0.0);
		}
	}

	@Override
	public boolean runDiagnostics() {
		if (Constants.ENABLE_DRIVE_TEST) {
			ConsoleReporter.report("Testing DRIVE---------------------------------");
			final double kLowCurrentThres = Constants.kDriveBaseTestLowCurrentThresh;
			final double kLowRpmThres = Constants.kDriveBaseTestLowRPMThresh;

			ArrayList<MotorDiagnostics> mAllMotorsDiagArr = new ArrayList<>();
			ArrayList<MotorDiagnostics> mLeftDiagArr = new ArrayList<>();
			ArrayList<MotorDiagnostics> mRightDiagArr = new ArrayList<>();
			mLeftDiagArr.add(new MotorDiagnostics("Drive Left Master", mLeftMaster));
			mLeftDiagArr.add(new MotorDiagnostics("Drive Left Slave 1", mLeftSlaveA, mLeftMaster));
//		mLeftDiagArr.add(new MotorDiagnostics("Drive Left Slave 2", mLeftSlaveB, mLeftMaster));
			mRightDiagArr.add(new MotorDiagnostics("Drive Right Master", mRightMaster));
			mRightDiagArr.add(new MotorDiagnostics("Drive Right Slave 1", mRightSlaveA, mRightMaster));
//		mRightDiagArr.add(new MotorDiagnostics("Drive Right Slave 2", mRightSlaveB, mRightMaster));

			mAllMotorsDiagArr.addAll(mLeftDiagArr);
			mAllMotorsDiagArr.addAll(mRightDiagArr);

			boolean failure = false;

			for (MotorDiagnostics mD : mAllMotorsDiagArr) {
				mD.setZero();
			}

			for (MotorDiagnostics mD : mAllMotorsDiagArr) {
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
			}

			if (mLeftDiagArr.size() > 0 && mRightDiagArr.size() > 0 && mAllMotorsDiagArr.size() > 0) {
				List<Double> leftMotorCurrents = mLeftDiagArr.stream().map(MotorDiagnostics::getMotorCurrent).collect(Collectors.toList());
				if (!Util.allCloseTo(leftMotorCurrents, leftMotorCurrents.get(0), Constants.kDriveBaseTestCurrentDelta)) {
					failure = true;
					ConsoleReporter.report("!!!!!!!!!!!!!!!!!! Drive Left2Cube Currents Different !!!!!!!!!!");
				}

				List<Double> rightMotorCurrents = mRightDiagArr.stream().map(MotorDiagnostics::getMotorCurrent).collect(Collectors.toList());
				if (!Util.allCloseTo(rightMotorCurrents, rightMotorCurrents.get(0), Constants.kDriveBaseTestCurrentDelta)) {
					failure = true;
					ConsoleReporter.report("!!!!!!!!!!!!!!!!!! Drive Right2Cube Currents Different !!!!!!!!!!");
				}

				List<Double> driveMotorRPMs = mAllMotorsDiagArr.stream().map(MotorDiagnostics::getMotorRPM).collect(Collectors.toList());
				if (!Util.allCloseTo(driveMotorRPMs, driveMotorRPMs.get(0), Constants.kDriveBaseTestRPMDelta)) {
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

	@Override
	public synchronized String generateReport() {

		//		sb.append("AccelX:" + mGyro.getRawAccelX() + ";");
//		sb.append("AccelY:" + mGyro.getRawAccelY() + ";");
//		sb.append("AccelZ:" + mGyro.getRawAccelZ() + ";");
//
//		sb.append("Gyro:" + mGyro.getRawYawDegrees() + ";");
//		sb.append("GyroRate:" + mGyro.getYawRateDegreesPerSec() + ";");

		//		sb.append("RobotPosition:" + PathFollowerRobotState.getInstance().getLatestFieldToVehicle().getValue().toString() + ";");

		return  "LeftDrivePos:" + mLeftMaster.getVelocity() + ";" +
				"LeftDriveVel:" + mLeftMaster.getVelocity() + ";" +
				"LeftDriveOutput:" + mPeriodicIO.left_demand + ";" +
				"LeftDrive1Current:" + mLeftMaster.getMCOutputCurrent() + ";" +
				"LeftDrive2Current:" + mLeftSlaveA.getMCOutputCurrent() + ";" +
				"LeftDrive3Current:" + mLeftSlaveB.getMCOutputCurrent() + ";" +
				"LeftDrive1HasReset:" + mLeftMaster.hasMotorControllerReset() + ";" +
				"LeftDrive2HasReset:" + mLeftSlaveA.hasMotorControllerReset() + ";" +
				"LeftDrive3HasReset:" + mLeftSlaveB.hasMotorControllerReset() + ";" +
				"LeftDriveOutputDutyCycle:" + mLeftMaster.getMCOutputPercent() + ";" +
				"LeftDriveOutputVoltage:" + mLeftMaster.getMCOutputPercent() * mLeftMaster.getMCInputVoltage() + ";" +
				"LeftDriveSupplyVoltage:" + mLeftMaster.getMCInputVoltage() + ";" +
				"RightDrivePos:" + mRightMaster.getVelocity() + ";" +
				"RightDriveVel:" + mRightMaster.getVelocity() + ";" +
				"RightDriveOutput:" + mPeriodicIO.right_demand + ";" +
				"RightDrive1Current:" + mRightMaster.getMCOutputCurrent() + ";" +
				"RightDrive2Current:" + mRightSlaveA.getMCOutputCurrent() + ";" +
				"RightDrive3Current:" + mRightSlaveB.getMCOutputCurrent() + ";" +
				"RightDrive1HasReset:" + mRightMaster.hasMotorControllerReset() + ";" +
				"RightDrive2HasReset:" + mRightSlaveA.hasMotorControllerReset() + ";" +
				"RightDrive3HasReset:" + mRightSlaveB.hasMotorControllerReset() + ";" +
				"RightDriveOutputDutyCycle:" + mRightMaster.getMCOutputPercent() + ";" +
				"RightDriveOutputVoltage:" + mRightMaster.getMCOutputPercent() * mRightMaster.getMCInputVoltage() + ";" +
				"RightDriveSupplyVoltage:" + mRightMaster.getBusVoltage() + ";" +
				"DriveMode:" + mDriveControlState.toString() + ";" +
				"IsDriveFaulted:" + isSystemFaulted() + ";";
	}

	@Override
	public synchronized boolean isSystemFaulted() {
		boolean leftSensorFaulted = !mLeftMaster.isEncoderPresent();
		boolean rightSensorFaulted = !mRightMaster.isEncoderPresent();
		boolean navXFaulted = !mGyro.isPresent();

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
		CLIMB
	}

	public enum ShifterState {
		FORCE_LOW_GEAR,
		FORCE_HIGH_GEAR,
		AUTO_SHIFT
	}

	public enum BrakeState {
		MOTOR_MASTER,
		MOTOR_SLAVEA,
		MOTOR_SLAVEB;
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
		public double gyro_pitch;
		public double gyro_roll;
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

