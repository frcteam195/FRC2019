package com.team195.lib.drivers;

import edu.wpi.first.wpilibj.Solenoid;

public class CKSolenoid extends Solenoid {
	public CKSolenoid(int solenoidChannel) {
		super((solenoidChannel >> 3) & 0x7, solenoidChannel & 0x7);
	}
}
