package com.team195.frc2019.subsystems.positions;

import com.team195.frc2019.subsystems.Turret;

public class TurretPositions {
	public static final double Home = 0;
	public static final double PositionDelta = 0.1;

	public static final double Left90 = Turret.convertTurretDegreesToRotations(-90);
	public static final double Right90 = Turret.convertTurretDegreesToRotations(90);
	public static final double Back180 = Turret.convertTurretDegreesToRotations(180);

	//Ball Shooter
	public static final double BallShootSpeedNormal = 1;
	public static final double BallShootSpeedIntake = -1;
	public static final double BallShootSpeedOff = 0;
}
