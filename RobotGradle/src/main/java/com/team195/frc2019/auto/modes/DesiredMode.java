package com.team195.frc2019.auto.modes;

import java.util.HashMap;

public enum DesiredMode {
	TwoHatchRocket(0),
	TwoHatchCargoship(1),
	CrossAutoLine(2),
	Characterization(3),
	DoNothing(4),
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
