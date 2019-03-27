package com.team195.frc2019.constants;

public class TestConstants {
	//////////////////////////////////////
	//TEST CONSTRAINTS
	public static final boolean RUN_INDIVIDUAL_TESTS = false;

	public static final boolean ENABLE_DRIVE_TEST = false;
	public static final double kDriveBaseTestLowCurrentThresh = 2;
	public static final double kDriveBaseTestLowRPMThresh = 100;
	public static final double kDriveBaseTestCurrentDelta = 5.0;
	public static final double kDriveBaseTestRPMDelta = 40.0;

	public static final boolean ENABLE_ELEVATOR_TEST = true;
	public static final double kElevatorTestLowCurrentThresh = 2;
	public static final double kElevatorTestLowRPMThresh = 15;
	public static final double kElevatorTestCurrentDelta = 5.0;
	public static final double kElevatorTestRPMDelta = 50.0;
	public static final double kElevatorTestSpeed = 0.5;
	public static final double kElevatorTestDuration = 1;

	public static final boolean ENABLE_BALL_INTAKE_ARM_TEST = false;
	public static final double kBallArmRotationTestLowCurrentThresh = 2;
	public static final double kBallArmRotationTestLowRPMThresh = 15;
	public static final double kBallArmRotationTestSpeed = 0.5;
	public static final double kBallArmRotationTestDurationDown = 2;
	public static final double kBallArmRotationTestDurationUp = 4;
	//////////////////////////////////////
}
