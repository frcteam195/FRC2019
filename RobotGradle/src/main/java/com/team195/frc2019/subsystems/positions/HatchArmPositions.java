package com.team195.frc2019.subsystems.positions;

import com.team195.frc2019.constants.CalConstants;

public class HatchArmPositions {
	public static final double Inside = 0;
	public static final double Outside = CalConstants.kHatchArmForwardSoftLimit;
//	public static final double Handoff = 0.274;
//	public static final double Handoff = 0.25;
//	public static final double Handoff = 0.32;
	public static final double Handoff = 0.345;
	public static final double CollisionThreshold = 0.377;
	public static final double PositionDelta = 0.05;


	public static final double RollerIntake = 1;
	public static final double RollerOuttake = -0.8;
	public static final double RollerOff = 0;
}
