package com.team195.lib.drivers;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class CKDoubleSolenoid extends DoubleSolenoid {

	private boolean mReversed = false;
	private boolean mCachedVal = false;

	public CKDoubleSolenoid(int solenoidForwardChannel) {
		super((solenoidForwardChannel >> 3) & 0x7, solenoidForwardChannel & 0x7, (solenoidForwardChannel & 0x7) + 1);
	}

	public void configReversed(boolean reversed) {
		mReversed = reversed;
	}

	public void set(boolean on) {
		mCachedVal = on;
		Value setVal;

		if (mReversed)
			setVal = on ? Value.kReverse : Value.kForward;
		else
			setVal = on ? Value.kForward : Value.kReverse;

		set(setVal);
	}

	public boolean isOn() {
		return mCachedVal;
	}

	public void turnOff() {
		set(Value.kOff);
	}
}
