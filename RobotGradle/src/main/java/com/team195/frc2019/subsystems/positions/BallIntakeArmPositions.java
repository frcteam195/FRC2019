package com.team195.frc2019.subsystems.positions;

import com.team195.frc2019.Constants;

public class BallIntakeArmPositions {
	public static final double Down = -3;
	public static final double Up = Constants.kBallIntakeArmForwardSoftLimit;
	public static final double CollisionThreshold = 0.07;
	public static final double PositionDelta = 0.005;

	//Ball Roller Setpoints
	public static final double RollerIntake = 1;
	public static final double RollerOuttake = -0.7;
	public static final double RollerOff = 0;
}
