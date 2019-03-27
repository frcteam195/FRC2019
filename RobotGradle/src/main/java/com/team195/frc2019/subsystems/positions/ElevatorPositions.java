package com.team195.frc2019.subsystems.positions;

import com.team195.frc2019.constants.CalConstants;

public class ElevatorPositions {
	//About 11.3261943987 inches per rotation

	public static final double Down = 0;
	public static final double BallHandoff = 0;
//	public static final double HatchHandoff = 0.6743;
	public static final double HatchHandoff = 0.66 * CalConstants.kNewPulleyFactor;
	public static final double CargoHatch = 0.3188 * CalConstants.kNewPulleyFactor;
	public static final double CargoBall = 2.2681 * CalConstants.kNewPulleyFactor;
	public static final double HatchPickupStation = CargoHatch * CalConstants.kNewPulleyFactor;
	public static final double HatchPickupStationLift = (HatchPickupStation + 0.3) * CalConstants.kNewPulleyFactor;
	public static final double Resting = CargoHatch * CalConstants.kNewPulleyFactor;
	public static final double RocketHatchLow = CargoHatch * CalConstants.kNewPulleyFactor;
	public static final double RocketHatchMed = 2.7358 * CalConstants.kNewPulleyFactor;
	public static final double RocketHatchHigh = 5.1638 * CalConstants.kNewPulleyFactor;
	public static final double RocketBallLow = 1.2031 * CalConstants.kNewPulleyFactor;
	public static final double RocketBallMed = 3.6074 * CalConstants.kNewPulleyFactor;
	public static final double RocketBallHigh = 6.0085 * CalConstants.kNewPulleyFactor;
	//Keep elevator here unless intaking a ball
	public static final double CollisionThresholdTurret = 0.3 * CalConstants.kNewPulleyFactor;
	public static final double CollisionThresholdHatchArm = 0.715 * CalConstants.kNewPulleyFactor;
	public static final double CollisionThresholdBallArm = 1.0 * CalConstants.kNewPulleyFactor;
	public static final double PositionDelta = 0.1 * CalConstants.kNewPulleyFactor;
	public static final double HatchLiftOffset = 0.5 * CalConstants.kNewPulleyFactor;
}
