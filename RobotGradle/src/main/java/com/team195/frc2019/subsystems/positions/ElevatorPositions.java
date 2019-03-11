package com.team195.frc2019.subsystems.positions;

import com.team195.frc2019.Constants;

public class ElevatorPositions {
	//About 11.3261943987 inches per rotation

	public static final double Down = 0;
	public static final double BallHandoff = 0;
//	public static final double HatchHandoff = 0.6743;
	public static final double HatchHandoff = 0.66 * Constants.kNewPulleyFactor;
	public static final double CargoHatch = 0.3188 * Constants.kNewPulleyFactor;
	public static final double CargoBall = 2.2681 * Constants.kNewPulleyFactor;
	public static final double HatchPickupStation = CargoHatch * Constants.kNewPulleyFactor;
	public static final double HatchPickupStationLift = (HatchPickupStation + 0.3) * Constants.kNewPulleyFactor;
	public static final double Resting = CargoHatch * Constants.kNewPulleyFactor;
	public static final double RocketHatchLow = CargoHatch * Constants.kNewPulleyFactor;
	public static final double RocketHatchMed = 2.7358 * Constants.kNewPulleyFactor;
	public static final double RocketHatchHigh = 5.1638 * Constants.kNewPulleyFactor;
	public static final double RocketBallLow = 1.2031 * Constants.kNewPulleyFactor;
	public static final double RocketBallMed = 3.6074 * Constants.kNewPulleyFactor;
	public static final double RocketBallHigh = 6.0085 * Constants.kNewPulleyFactor;
	//Keep elevator here unless intaking a ball
	public static final double CollisionThresholdTurret = 0.3 * Constants.kNewPulleyFactor;
	public static final double CollisionThresholdHatchArm = 0.715 * Constants.kNewPulleyFactor;
	public static final double CollisionThresholdBallArm = 1.0 * Constants.kNewPulleyFactor;
	public static final double PositionDelta = 0.1 * Constants.kNewPulleyFactor;
	public static final double HatchLiftOffset = 0.5 * Constants.kNewPulleyFactor;
}
