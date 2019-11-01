package org.aceshigh176.lib.robotbase;

import com.team195.lib.drivers.motorcontrol.MCControlMode;

import java.util.HashMap;

public enum RobotOperationalMode {
    kDisabled(0),
    kAutonomous(1),
    kTeleop(2),
    kTest(3);

	public final int value;

	private static HashMap<Integer, RobotOperationalMode> intLookupMap = new HashMap<>();

	static {
		for (RobotOperationalMode type : RobotOperationalMode.values()) {
			intLookupMap.put(type.value, type);
		}
	}

	public static RobotOperationalMode valueOf(Object value) {
		RobotOperationalMode retval = null;

		if (value instanceof Number) {
			retval = intLookupMap.get((int)value);
		}

		if (retval != null)
			return retval;
		return kDisabled;
	}
	RobotOperationalMode(int initValue) {
		this.value = initValue;
	}
	public String toString() {
		return valueOf(value).name();
	}
}
