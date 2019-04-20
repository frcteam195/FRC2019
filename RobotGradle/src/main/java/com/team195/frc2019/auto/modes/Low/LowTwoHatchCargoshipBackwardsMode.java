package com.team195.frc2019.auto.modes.Low;

import com.team195.frc2019.auto.AutoModeBase;
import com.team195.frc2019.auto.AutoModeEndedException;
import com.team195.frc2019.auto.actions.*;
import com.team195.frc2019.auto.autonomy.AutomatedActions;
import com.team195.frc2019.paths.TrajectoryGenerator;
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

		lowStartToSideCargoForwardFacing = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().lowStartToSideCargoForwardFacing.get(mStartedLeft), true);
		sideCargoForwardFacingToFeederStation = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().sideCargoForwardFacingToFeederStation.get(mStartedLeft));
		feederStationToFrontCargoHatchForward = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().feederStationToFrontCargoHatchForward.get(mStartedLeft));
		frontCargoHatchForwardToFeederStation = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().frontCargoHatchForwardToFeederStation.get(mStartedLeft));
	}

	@Override
	public void done() {
		Drive.getInstance().setVelocity(DriveSignal.NEUTRAL, DriveSignal.NEUTRAL);
	}

	@Override
	protected void routine() throws AutoModeEndedException {
		runAction(new ParallelAction(lowStartToSideCargoForwardFacing,
				new SeriesAction(new WaitAction(2.0), AutomatedActions.setTurretPosition(mStartedLeft ? TurretPositions.Left90 : TurretPositions.Right90))));
		runAction(AutomatedActions.placeHatch());
		runAction(new ParallelAction(sideCargoForwardFacingToFeederStation,
				new SeriesAction(new WaitUntilInsideRegion(new Translation2d(0, -160),
						new Translation2d(50, -110), mStartedLeft),
						AutomatedActions.pickupHatchFeederStation())));
		runAction(new ParallelAction(feederStationToFrontCargoHatchForward,
				new SeriesAction(AutomatedActions.setTurretPosition(TurretPositions.Back180),
						new WaitAction(0.5),
						AutomatedActions.ballArmSet(BallIntakeArmPositions.Up)
						)));
		runAction(AutomatedActions.placeHatch());
		runAction(new ParallelAction(frontCargoHatchForwardToFeederStation,
				new SeriesAction(AutomatedActions.ballArmSet(BallIntakeArmPositions.Down),
						new WaitAction(0.5),
						AutomatedActions.setTurretPosition(TurretPositions.Home)
				)));
	}
}
