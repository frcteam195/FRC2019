package com.team195.lib.util;

import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.reporters.MessageLevel;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;

public class ShiftHelper {
	private Solenoid mSolenoid;
	private DoubleSolenoid mDoubleSolenoid;
	private boolean isDoubleSolenoid;
	private boolean highGearSingleSolValue;
	private DoubleSolenoid.Value highGearDoubleSolValue = DoubleSolenoid.Value.kOff;

	public ShiftHelper(int solenoidId) {
		isDoubleSolenoid = false;
		mSolenoid = new Solenoid(solenoidId);
		highGearSingleSolValue = true;
	}

	public ShiftHelper(int doubleSolenoidID1, int doubleSolenoidID2) {
		isDoubleSolenoid = true;
		mDoubleSolenoid = new DoubleSolenoid(doubleSolenoidID1, doubleSolenoidID2);
	}

	public void configHighGear(boolean highIsTrueOrForward) {
		if (!isDoubleSolenoid)
			highGearSingleSolValue = highIsTrueOrForward;
		else
			highGearDoubleSolValue = highIsTrueOrForward ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse;
	}

	public void shift(boolean highGear) {
		if (isDoubleSolenoid) {
			switch (highGearDoubleSolValue) {
				case kForward:
					mDoubleSolenoid.set(highGear ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
					break;
				case kReverse:
					mDoubleSolenoid.set(highGear ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward);
					break;
				default:
					ConsoleReporter.report("You forgot to configHighGear for the ShiftHelper!", MessageLevel.ERROR);
					break;
			}
		} else {
			mSolenoid.set(highGear == highGearSingleSolValue);
		}
	}
}
