package com.team195.frc2019.subsystems.positions;

import com.team195.frc2019.constants.CalConstants;

public class ElevatorPositions {
	//About 11.3261943987 inches per rotation

	public static final double Down = 0;
	public static final double BallHandoff = 0;
//	public static final double CargoHatch = 0.416015625;
public static final double CargoHatch = 0.40;
	public static final double CargoBall = 2.2681 * CalConstants.kNewPulleyFactor;
	public static final double HatchPickupStation = CargoHatch;
	public static final double HatchPickupStationLift = HatchPickupStation + 0.2;
	public static final double Resting = CargoHatch;
	public static final double RocketHatchLow = CargoHatch;
	public static final double RocketHatchMed = 3.3857421875;
	public static final double RocketHatchHigh = 6.27587890625;
	public static final double RocketBallLow = 1.2031 * CalConstants.kNewPulleyFactor;
	public static final double RocketBallMed = 3.6074 * CalConstants.kNewPulleyFactor;
	public static final double RocketBallHigh = 6.0085 * CalConstants.kNewPulleyFactor;
	//Keep elevator here unless intaking a ball
	public static final double CollisionThresholdTurret = 0.25 * CalConstants.kNewPulleyFactor;
	public static final double CollisionThresholdHatchArm = 0.715 * CalConstants.kNewPulleyFactor;
	public static final double CollisionThresholdBallArm = 0.75 * CalConstants.kNewPulleyFactor;
	public static final double PositionDelta = 0.1 * CalConstants.kNewPulleyFactor;
}
