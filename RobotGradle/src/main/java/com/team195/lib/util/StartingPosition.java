package com.team195.lib.util;

public enum StartingPosition {
	Left(0),
	Center(1),
	Right(2),
	Invalid(30);

	public final int value;
	StartingPosition(int initValue)
	{
		this.value = initValue;
	}

	public static StartingPosition valueOf(int value) {
		switch (value) {
			case 0:
				return Left;
			case 1:
				return Center;
			case 2:
				return Right;
			default:
				return Invalid;
		}
	}
}
