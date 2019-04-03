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
	public static final double kRobotAngularInertia = 64.5986;  // kg m^2 TODO tune
	public static final double kRobotAngularDrag = 21.0;  // N*m / (rad/sec) TODO tune
	public static final double kDriveVIntercept = 0.23319967; //0.781046438 angular  // V
	public static final double kDriveKv = 0.1825403887;  // V per rad/s
	public static final double kDriveKa = 0.005275085;  // V per rad/s^2


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

	// PID gains for drive velocity loop (LOW GEAR)
	// Units: setpoint, error, and output are in ticks per second.
	public static final double kDriveLowGearVelocityKp = 0.0003;
	public static final double kDriveLowGearVelocityKi = 0.0;
	public static final double kDriveLowGearVelocityKd = 0.0007;
	public static final double kDriveLowGearVelocityKf = 0;//0.000176;
	public static final double kDriveLowGearVelocityDFilter = 0.8;
	public static final int kDriveLowGearVelocityIZone = 0;
	public static final double kDriveVoltageRampRate = 0.0;

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
	public static final int kDriveLeftClimbCurrentLim = 50;
	public static final int kDriveRightClimbCurrentLim = 35;

	//17.25:1
	public static final double kElevatorPositionKp = 1.9;
	public static final double kElevatorPositionKi = 0.0;
	public static final double kElevatorPositionKd = 8.0;
	public static final double kElevatorPositionKf = 0.1380124477;
	public static final int kElevatorPositionCruiseVel = 850;
	public static final int kElevatorPositionMMAccel = 600;
	//Units in rotations
	public static final double kNewPulleyFactor = 1.25;
	public static final double kElevatorPositionForwardSoftLimit = 6.3 * kNewPulleyFactor;
	public static final double kElevatorPositionReverseSoftLimit = 0;
	public static final double kElevatorLowSensitivityThreshold = 2.5;
	public static final double kLowSensitivityFactor = 0.5;

	//50:1
	public static final double kTurretPositionKp = 4.3;
	public static final double kTurretPositionKi = 0.0;
	public static final double kTurretPositionKd = 8.0;
	public static final double kTurretPositionKf = 0.400360;
	public static final int kTurretPositionCruiseVel = 350;
	public static final int kTurretPositionMMAccel = 600;
	//Units in rotations
	public static final double kTurretForwardSoftLimit = Turret.convertTurretDegreesToRotations(225);
	public static final double kTurretReverseSoftLimit = -kTurretForwardSoftLimit;
	public static final double kTurretSmallGearTeeth = 36;
	public static final double kTurretLargeGearTeeth = 252;

	//TODO: Tune
	//100:1
	public static final double kHatchArmPositionKp = 2.4;
	public static final double kHatchArmPositionKi = 0.0;
	public static final double kHatchArmPositionKd = 6.0;
	public static final double kHatchArmPositionKf = 0.8000721603;
	public static final int kHatchArmPositionCruiseVel = 100;
	public static final int kHatchArmPositionMMAccel = 60;
	//Units in rotations
	//    public static final double kHatchArmForwardSoftLimit = 0.6174;
		// public static final double kHatchArmForwardSoftLimit = 0.622;
	public static final double kHatchArmForwardSoftLimit = 0.700;
	public static final double kHatchArmReverseSoftLimit = 0;

	public static final double kBallIntakeArmUpPositionKp = 3.7;
	public static final double kBallIntakeArmUpPositionKi = 0.0;
	public static final double kBallIntakeArmUpPositionKd = 12.0;
	public static final double kBallIntakeArmUpPositionKf = 0.8000721603;
	public static final int kBallIntakeArmUpPositionCruiseVel = 160;
	public static final int kBallIntakeArmUpPositionMMAccel = 320;

	//200:1
//    public static final double kBallIntakeArmUpPositionKp = 7.3;
//    public static final double kBallIntakeArmUpPositionKi = 0.0;
//    public static final double kBallIntakeArmUpPositionKd = 100.0;
//    public static final double kBallIntakeArmUpPositionKf = 1.6;
//    public static final int kBallIntakeArmUpPositionCruiseVel = 50;
//    public static final int kBallIntakeArmUpPositionMMAccel = 45;

	//Units in rotations
//    public static final double kBallIntakeArmForwardSoftLimit = 0.174;

	//100:1
	public static final double kBallIntakeArmDownPositionKp = 3.7;
	public static final double kBallIntakeArmDownPositionKi = 0.0;
	public static final double kBallIntakeArmDownPositionKd = 12.0;
	public static final double kBallIntakeArmDownPositionKf = 0.8000721603;
	public static final int kBallIntakeArmDownPositionCruiseVel = 160;
	public static final int kBallIntakeArmDownPositionMMAccel = 320;
	public static final double kBallIntakeArmForwardSoftLimit = 4;


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
