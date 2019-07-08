package com.team195.frc2019.constants;

import com.team195.frc2019.subsystems.Turret;

public class CalConstants {
	/* ROBOT PHYSICAL CONSTANTS */

	// Wheels
	public static final double kDriveWheelTrackWidthInches = 23.5;
	public static final double kDriveWheelDiameterInches = 5.0 * 0.9625;
	public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
	public static final double kTrackScrubFactor = 1.0;  // Tune me!

	// Tuned dynamics
	public static final double kRobotLinearInertia = 68.946;  // kg TODO tune
	public static final double kRobotAngularInertia = 125;  // kg m^2 TODO tune
	public static final double kRobotAngularDrag = 0.1;  // N*m / (rad/sec) TODO tune
	public static final double kDriveVIntercept = 0.30165000; //0.781046438 angular  // V
	public static final double kDriveKv = 0.186163041;  // V per rad/s
	public static final double kDriveKa = 0.0086739979;  // V per rad/s^2


	// Pose of the LIDAR frame w.r.t. the robot frame
	// TODO measure in CAD/on robot!
	public static final double kLidarXOffset = -3.3211;
	public static final double kLidarYOffset = 0.0;
	public static final double kLidarYawAngleDegrees = 0.0;

	// Gearing and mechanical constants.
	public static final double kDriveDownShiftVelocity = 9.5 * 12.0;  // inches per second
	public static final double kDriveDownShiftAngularVelocity = Math.PI / 2.0; // rad/sec
	public static final double kDriveUpShiftVelocity = 11.0 * 12.0;  // inches per second
	public static final double kPathKX = 4.0;  // units/s per unit of error
	public static final double kPathLookaheadTime = 0.4;  // seconds to look ahead along the path for steering
	public static final double kPathMinLookaheadDistance = 24.0;  // inches

	public static final double kDriveGearRatioMotorConversionFactor = 8.0;

	/* CONTROL LOOP GAINS */

	///////////////////////////////////////////////////////////////////////////
	//Drive
	public static final double kDriveDefaultVoltageCompensationSetpoint = 12.0;
	public static final double kDriveDefaultOpenLoopRampRate = 0.1;

	// PID gains for drive velocity loop (LOW GEAR)
	// Units: setpoint, error, and output are in ticks per second.
	public static final double kDriveLowGearVelocityKp = 0.002;
	public static final double kDriveLowGearVelocityKi = 0.0;
	public static final double kDriveLowGearVelocityKd = 0.000;
	public static final double kDriveLowGearVelocityKf = 0;//0.000176;
	public static final double kDriveLowGearVelocityDFilter = 1;
	public static final int kDriveLowGearVelocityIZone = 0;
	public static final double kDriveVoltageRampRate = 0.1;

	// PID gains for drive position loop (LOW GEAR)
	// Units: setpoint, error, and output are in ticks per second.
	public static final double kDriveLowGearPositionKp = 0.000090;
	public static final double kDriveLowGearPositionKi = 0.0;
	public static final double kDriveLowGearPositionKd = 0.001600;
	public static final double kDriveLowGearPositionDFilter = 0.25;
	public static final double kDriveLowGearPositionKf = 0.000162;
	public static final double kDriveLowGearPositionCruiseVel = 2000;
	public static final double kDriveLowGearPositionAccel = 1000;
	public static final int kDriveLowGearPositoinIZone = 0;
	public static final int kDriveLowGearCurrentLim = 50;
	public static final int kDriveLeftClimbCurrentLim = 65;
	public static final int kDriveRightClimbCurrentLim = 80;
	public static final int kDriveLeftRetractCurrentLim = 50;
	///////////////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////////////
	//Elevator
	//17.25:1
	public static final double kElevatorPositionKp = 1.9;
	public static final double kElevatorPositionKi = 0.0;
	public static final double kElevatorPositionKd = 8.0;
	public static final double kElevatorPositionKf = 0.1380124477;
	public static final int kElevatorPositionCruiseVel = 850;
	public static final int kElevatorPositionMMAccel = 900;
	public static final int kElevatorPositionSCurveStrength = 5;
	public static final int kElevatorContinuousCurrentLimit = 15;
	public static final int kElevatorPeakCurrentThreshold = 20;
	public static final int kElevatorPeakCurrentThresholdExceedDuration = 0; //250;
	//Units in rotations
	public static final double kNewPulleyFactor = 1.25;
	public static final double kElevatorPositionForwardSoftLimit = 6.3 * kNewPulleyFactor;
	public static final double kElevatorPositionReverseSoftLimit = 0;
	public static final double kElevatorLowSensitivityThreshold = 2.5;
	public static final double kElevatorLowSensitivityFactor = 0.5;
	///////////////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////////////
	//Turret
	//50:1
	public static final double kTurretPositionKp = 4.3;
	public static final double kTurretPositionKi = 0.0;
	public static final double kTurretPositionKd = 8.0;
	public static final double kTurretPositionKf = 0.400360;
	public static final int kTurretPositionCruiseVel = 350;
	public static final int kTurretPositionMMAccel = 600;
	public static final int kTurretPositionSCurveStrength = 5;
	public static final int kTurretContinuousCurrentLimit = 5;
	public static final int kTurretPeakCurrentThreshold = 7;
	public static final int kTurretPeakCurrentThresholdExceedDuration = 150;
	public static final double kTurretBallShooterOpenLoopRamp = 0.2;
	public static final int kTurretBallShooterContinuousCurrentLimit = 15;
	public static final int kTurretBallShooterPeakCurrentThreshold = 25;
	public static final int kTurretBallShooterPeakCurrentThresholdExceedDuration = 450;
	//Units in rotations
	public static final double kTurretForwardSoftLimit = Turret.convertTurretDegreesToRotations(225);
	public static final double kTurretReverseSoftLimit = -kTurretForwardSoftLimit;
	public static final double kTurretSmallGearTeeth = 36;
	public static final double kTurretLargeGearTeeth = 252;
	///////////////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////////////
	//Ball Intake Arm
	public static final double kBallIntakeArmUpPositionKp = 3.7;
	public static final double kBallIntakeArmUpPositionKi = 0.0;
	public static final double kBallIntakeArmUpPositionKd = 12.0;
	public static final double kBallIntakeArmUpPositionKf = 0.5600505122;
	public static final int kBallIntakeArmUpPositionCruiseVel = 220;
	public static final int kBallIntakeArmUpPositionMMAccel = 420;
	public static final int kBallArmRotationContinuousCurrentLimit = 10;
	public static final int kBallArmRotationPeakCurrentThreshold = 12;
	public static final int kBallArmRotationPeakCurrentThresholdExceedDuration = 200;
	public static final double kBallArmRollerOpenLoopRamp = 0.2;
	public static final int kBallArmRollerContinuousCurrentLimit = 30;
	public static final int kBallArmRollerPeakCurrentThreshold = 31;
	public static final int kBallArmRollerPeakCurrentThresholdExceedDuration = 100;
	///////////////////////////////////////////////////////////////////////////

	//Breaker model for trip time output in seconds y = a*(current_percent_over_rating)^b + c
	public static final double kPDPBreakerModelA = 282.2962;
	public static final double kPDPBreakerModelB = -6.6305;
	public static final double kPDPBreakerModelC = 0.5;
	public static final double kPDPDefaultSafetyFactor = 4.0;

	//TODO: Tune collision detection
	// Collision Detection
	public static final double kCollisionDetectionJerkThreshold = 950;
	public static final double kTippingThresholdDeg = 11;
}
