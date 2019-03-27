package com.team195.lib.util;

public enum StartingPosition {
	LeftLow(0),
	LeftHigh(1),
	Center(2),
	RightLow(3),
	RightHigh(4),
	Invalid(30);

	public final int value;
	StartingPosition(int initValue)
	{
		this.value = initValue;
	}

	public static StartingPosition valueOf(int value) {
		switch (value) {
			case 0:
				return LeftLow;
			case 1:
				return LeftHigh;
			case 2:
				return Center;
			case 3:
				return RightLow;
			case 4:
				return RightHigh;
			default:
				return Invalid;
		}
	}
}
