package com.team195.frc2019.auto.modes.Low;

import com.team195.frc2019.auto.AutoModeBase;
import com.team195.frc2019.auto.AutoModeEndedException;
import com.team195.frc2019.auto.actions.*;
import com.team195.frc2019.auto.autonomy.AutomatedActions;
import com.team195.frc2019.paths.LowLow.Path1;
import com.team195.frc2019.paths.LowLow.Path2;
import com.team195.frc2019.paths.LowLow.Path3;
import com.team195.frc2019.paths.LowLow.Path4;
import com.team195.frc2019.paths.TrajectoryGenerator;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.subsystems.Drive;
import com.team195.frc2019.subsystems.positions.BallIntakeArmPositions;
import com.team195.frc2019.subsystems.positions.TurretPositions;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.DriveSignal;

public class LowTwoHatchCargoshipBackwardsMode extends AutoModeBase {
	private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();

	private final DriveTrajectory lowStartToSideCargoForwardFacing;
	private final DriveTrajectory sideCargoForwardFacingToFeederStation;
	private final DriveTrajectory feederStationToFrontCargoHatchForward;
	private final DriveTrajectory frontCargoHatchForwardToFeederStation;

	private final boolean mStartedLeft;

	public LowTwoHatchCargoshipBackwardsMode(boolean robotStartedOnLeft) {
		mStartedLeft = robotStartedOnLeft;

		lowStartToSideCargoForwardFacing = new DriveTrajectory(new Path1(), true);
		sideCargoForwardFacingToFeederStation = new DriveTrajectory(new Path2());
		feederStationToFrontCargoHatchForward = new DriveTrajectory(new Path3());
		frontCargoHatchForwardToFeederStation = new DriveTrajectory(new Path4());
	}

	@Override
	public void done() {
		Drive.getInstance().setVelocity(DriveSignal.NEUTRAL, DriveSignal.NEUTRAL);
	}

	@Override
	protected void routine() throws AutoModeEndedException {
		ConsoleReporter.report("Two Hatch Cargo Auto Mode");
		runAction(new ParallelAction(lowStartToSideCargoForwardFacing,
				new SeriesAction(new WaitAction(0.35), AutomatedActions.setTurretPosition(!mStartedLeft ? TurretPositions.Left90 : TurretPositions.Right90))));
		runAction(AutomatedActions.placeHatchAuto());
		ConsoleReporter.report("Completed First Path");
		runAction(new ParallelAction(sideCargoForwardFacingToFeederStation, AutomatedActions.setTurretPosition(TurretPositions.Home),
				new SeriesAction(new WaitUntilInsideRegion(new Translation2d(0, 0),
						new Translation2d(95, 70), mStartedLeft),
						AutomatedActions.pickupHatchFeederStation())));
		runAction(new ParallelAction(feederStationToFrontCargoHatchForward,
				new SeriesAction(AutomatedActions.setTurretPosition(TurretPositions.Back180),
						new WaitAction(0.5),
						AutomatedActions.ballArmSet(BallIntakeArmPositions.Up)
						)));
		runAction(AutomatedActions.placeHatchAuto());
		runAction(new ParallelAction(frontCargoHatchForwardToFeederStation,
				new SeriesAction(AutomatedActions.ballArmSet(BallIntakeArmPositions.Down),
						new WaitAction(0.5),
						AutomatedActions.setTurretPosition(TurretPositions.Home)
				)));
	}
}
