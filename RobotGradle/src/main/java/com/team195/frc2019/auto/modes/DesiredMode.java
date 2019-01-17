package com.team195.frc2019.auto.modes;

import java.util.HashMap;

public enum DesiredMode {
	CrossAutoLine(0),
	Characterization(1),
	DoNothing(2),
	Invalid(30);

	public final int value;
	DesiredMode(int initValue)
	{
		this.value = initValue;
	}

	private static HashMap<Integer, DesiredMode> intLookupMap = new HashMap<>();

	static {
		for (DesiredMode type : DesiredMode.values()) {
			intLookupMap.put(type.value, type);
		}
	}

	public static DesiredMode valueOf(int value) {
		DesiredMode retval = intLookupMap.get(value);
		if (retval != null)
			return retval;
		return Invalid;
	}
}
