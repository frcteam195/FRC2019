package com.team195.lib.drivers;

import edu.wpi.first.wpilibj.Solenoid;

public class CKSolenoid extends Solenoid {
	private boolean mInverted = false;

	public CKSolenoid(int solenoidChannel) {
		super((solenoidChannel >> 3) & 0x7, solenoidChannel & 0x7);
	}

	public void setInverted(boolean inverted) {
		mInverted = inverted;
	}

	@Override
	public void set(boolean on) {
		if (mInverted)
			super.set(!on);
		else
			super.set(on);
	}
}
