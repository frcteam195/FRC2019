package com.team195.frc2019;

import com.team195.lib.drivers.dashjoy.CKDashJoystick;

public class Controllers {
	private CKDashJoystick driveJoystick;
	private CKDashJoystick armControlJoystick;
	private CKDashJoystick buttonBox1;
	private CKDashJoystick buttonBox2;
	
	private static Controllers instance = null;
	
	public Controllers() {
		driveJoystick = new CKDashJoystick(0);
		armControlJoystick = new CKDashJoystick(1);
		buttonBox1 = new CKDashJoystick(2);
		buttonBox2 = new CKDashJoystick(3);
	}
	
	public static Controllers getInstance() {
		if(instance == null)
			instance = new Controllers();
		
		return instance;
	}

	public CKDashJoystick getDriveJoystick() {
		return driveJoystick;
	}

	public CKDashJoystick getArmControlJoystick() {
		return armControlJoystick;
	}

	public CKDashJoystick getButtonBox1() {
		return buttonBox1;
	}

	public CKDashJoystick getButtonBox2() {
		return buttonBox2;
	}
}