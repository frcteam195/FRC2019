package com.team195.frc2019.constants;

public class DeviceIDConstants {
	// Do not change anything after this line unless you rewire the robot and
	// update the spreadsheet!
	// Port assignments should match up with the spreadsheet here:
	// https://docs.google.com/spreadsheets/d/179YszqnEWPWInuHUrYJnYL48LUL7LUhZrnvmNu1kujE/edit#gid=0

	/* I/O */
	// (Note that if multiple talons are dedicated to a mechanism, any sensors
	// are attached to the master)

	// Drive
	public static final int kRightDriveMasterId = 1;
	public static final int kRightDriveSlaveAId = 2;
	public static final int kRightDriveSlaveBId = 3;
	public static final int kLeftDriveMasterId = 4;
	public static final int kLeftDriveSlaveAId = 5;
	public static final int kLeftDriveSlaveBId = 6;


	// Elevator
	public static final int kElevatorMasterLeftId = 8;
	public static final int kElevatorSlaveALeftId = 9;
	public static final int kElevatorSlaveBRightId = 7;
	public static final int kElevatorSlaveCRightId = 16;


	//Ball Intake Arm
	public static final int kBallIntakeRotationMotorId = 10;
	public static final int kBallIntakeRollerMotorId = 11;


	//Hatch Intake Arm
	public static final int kHatchIntakeRotationMotorId = 12;
	public static final int kHatchIntakeRollerMotorId = 13;


	//Turret
	public static final int kTurretMotorId = 14;
	public static final int kBallShooterMotorId = 15;


	// Solenoids
	public static final int kPTOShifterSolenoidId = 0;
	public static final int kBallIntakeBarSolenoidId = 2;
	//    public static final int kHatchBeakSolenoidId = 3;
	//    public static final int kHatchPushSolenoidId = 5;
	//    public static final int kBallPushSolenoidId = 7;
	public static final int kHatchBeakSolenoidId = 9;
	public static final int kHatchBeakFeedSolenoidId = 10;
	public static final int kHatchPushSolenoidId = 8;
	public static final int kBallPushSolenoidId = 11;


	public static final int kCANifierLEDId = 30;
}
