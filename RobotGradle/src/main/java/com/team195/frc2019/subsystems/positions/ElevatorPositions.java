package com.team195.frc2019.subsystems.positions;

public class ElevatorPositions {
	//About 11.3261943987 inches per rotation

	public static final double Down = 0;
	public static final double BallHandoff = 0;
//	public static final double HatchHandoff = 0.6743;
	public static final double HatchHandoff = 0.66;
	public static final double CargoHatch = 0.3188;
	public static final double CargoBall = 2.2681;
	public static final double HatchPickupStation = CargoHatch;
	public static final double Resting = CargoHatch;
	public static final double RocketHatchLow = CargoHatch;
	public static final double RocketHatchMed = 2.7358;
	public static final double RocketHatchHigh = 5.1638;
	public static final double RocketBallLow = 1.2031;
	public static final double RocketBallMed = 3.6074;
	public static final double RocketBallHigh = 6.0085;
	//Keep elevator here unless intaking a ball
	public static final double CollisionThresholdTurret = 0.3;
	public static final double CollisionThresholdHatchArm = 0.715;
	public static final double CollisionThresholdBallArm = 1.0;
	public static final double PositionDelta = 0.1;
	public static final double HatchLiftOffset = 0.5;
}
