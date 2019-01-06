package com.team195.frc2019;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.reporters.MessageLevel;
import com.team195.lib.util.ShiftHelper;
import com.team195.lib.util.drivers.*;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;

public class Controllers {
	private Compressor compressor;
	private PowerDistributionPanel powerDistributionPanel;
	private KnightJoystick driveJoystickThrottle;
	private KnightJoystick driveJoystickWheel;
	private KnightJoystick armControlJoystick;
	private KnightJoystick buttonBox1;
	private KnightJoystick buttonBox2;
	private CKTalonSRX leftDrive1;
	private BaseMotorController leftDrive2;
	private BaseMotorController leftDrive3;
	private CKTalonSRX rightDrive1;
	private BaseMotorController rightDrive2;
	private BaseMotorController rightDrive3;

	private CKTalonSRX arm1Motor;

	private CKTalonSRX elevatorMotorMaster;
	private BaseMotorController elevatorMotorSlave;
	private BaseMotorController elevatorMotorSlave2;
	private BaseMotorController elevatorMotorSlave3;
	private TalonSRX intakeMotor;
	private TalonSRX intake2Motor;
	private CKTalonSRX climberMotorMaster;
	private TalonSRX climberMotorSlave;

	private ShiftHelper shiftHelper = null;
	private Solenoid intakeSolenoid;
	private Solenoid climberLockSolenoid;

	private CANifier canifierLED;
//	private PigeonDriver pigeonIMU;

//	private DigitalOutput rLED;
//	private DigitalOutput gLED;
//	private DigitalOutput bLED;

	private KnightDigitalInput elevatorHomeSwitch;
	private KnightDigitalInput cubeSensor;

	private NavX navX;
	
	private static Controllers instance = null;
	
	public Controllers() {
		compressor = new Compressor();
		powerDistributionPanel = new PowerDistributionPanel();

		//Drive Joystick Setup
		driveJoystickThrottle = new KnightJoystick(0);
		//driveJoystickWheel = new KnightJoystick(1);
		armControlJoystick = new KnightJoystick(1);
		buttonBox1 = new KnightJoystick(2);
		buttonBox2 = new KnightJoystick(3);

		canifierLED = new CANifier(Constants.kCANifierLEDId);
//		pigeonIMU = new PigeonDriver(2);

	    try {
	        navX = new NavX(SPI.Port.kMXP);
	    } catch (Exception ex) {
			ConsoleReporter.report(ex, MessageLevel.DEFCON1);
	    }
	}
	
	public static Controllers getInstance() {
		if(instance == null)
			instance = new Controllers();
		
		return instance;
	}
	
	public KnightJoystick getDriveJoystickThrottle() {
		return driveJoystickThrottle;
	}

	public KnightJoystick getDriveJoystickWheel() {
		return driveJoystickWheel;
	}

	public KnightJoystick getArmControlJoystick() {
		return armControlJoystick;
	}

	public KnightJoystick getButtonBox1() { return buttonBox1; }

	public KnightJoystick getButtonBox2() { return buttonBox2; }

	public CKTalonSRX getLeftDrive1() {
		return leftDrive1;
	}
	
	public BaseMotorController getLeftDrive2() {
		return leftDrive2;
	}
	
	public BaseMotorController getLeftDrive3() {
		return leftDrive3;
	}
	
	public CKTalonSRX getRightDrive1() {
		return rightDrive1;
	}
	
	public BaseMotorController getRightDrive2() {
		return rightDrive2;
	}
	
	public BaseMotorController getRightDrive3() {
		return rightDrive3;
	}

	public CKTalonSRX getArm1Motor() {
		return arm1Motor;
	}

	public TalonSRX getIntake2Motor() {
		return intake2Motor;
	}

	public TalonSRX getIntakeMotor() {
		return intakeMotor;
	}

	public CKTalonSRX getElevatorMotorMaster() {
		return elevatorMotorMaster;
	}
	
	public BaseMotorController getElevatorMotorSlave() {
		return elevatorMotorSlave;
	}

	public BaseMotorController getElevatorMotorSlave2() {
		return elevatorMotorSlave2;
	}

	public BaseMotorController getElevatorMotorSlave3() {
		return elevatorMotorSlave3;
	}

	public CKTalonSRX getClimberMotorMaster() {
		return climberMotorMaster;
	}

	public TalonSRX getClimberMotorSlave() {
		return climberMotorSlave;
	}

	public NavX	getNavX() {
		return navX;
	}

	public CANifier getCANifierLED() {
		return canifierLED;
	}

//	public DigitalOutput getRedLED() {
//		return rLED;
//	}
//
//	public DigitalOutput getGreenLED() {
//		return gLED;
//	}
//
//	public DigitalOutput getBlueLED() {
//		return bLED;
//	}

	public PowerDistributionPanel getPowerDistributionPanel() {
		return powerDistributionPanel;
	}

	public Compressor getCompressor() {
		return compressor;
	}

	public ShiftHelper getShiftHelper() {
		return shiftHelper;
	}

	public Solenoid getIntakeSolenoid() {
		return intakeSolenoid;
	}

	public Solenoid getClimberLockSolenoid() {
		return climberLockSolenoid;
	}

	public KnightDigitalInput getElevatorHomeSwitch() {
		return elevatorHomeSwitch;
	}

	public KnightDigitalInput getCubeSensor() {
		return cubeSensor;
	}

//	public PigeonDriver getPigeonIMU() {
//		return pigeonIMU;
//	}
}