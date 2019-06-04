package com.team195.frc2019.auto.modes.High;

import com.team195.frc2019.auto.AutoModeBase;
import com.team195.frc2019.auto.AutoModeEndedException;
import com.team195.frc2019.auto.actions.DriveTrajectory;
import com.team195.frc2019.auto.actions.ParallelAction;
import com.team195.frc2019.auto.actions.SeriesAction;
import com.team195.frc2019.auto.actions.WaitUntilInsideRegion;
import com.team195.frc2019.auto.autonomy.AutomatedActions;
import com.team195.frc2019.paths.TrajectoryGenerator;
import com.team195.frc2019.subsystems.Drive;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.DriveSignal;

public class HighTwoHatchRocketMode extends AutoModeBase {
	private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();

	private final DriveTrajectory highStartToCloseRocketFarHatch;
	private final DriveTrajectory closeRocketFarHatchToTurn1;
	private final DriveTrajectory closeRocketFarHatchTurn1ToFeederStation;
	private final DriveTrajectory closeRocketFeederStationToTurn2;
	private final DriveTrajectory closeRocketTurn2ToCloseHatch;
	private final DriveTrajectory closeRocketCloseHatchToBall;

	private final boolean mStartedLeft;

	public HighTwoHatchRocketMode(boolean robotStartedOnLeft) {
		mStartedLeft = robotStartedOnLeft;

		highStartToCloseRocketFarHatch = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().highStartToCloseRocketFarHatch.get(mStartedLeft), true);
		closeRocketFarHatchToTurn1 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().closeRocketFarHatchToTurn1.get(mStartedLeft));
		closeRocketFarHatchTurn1ToFeederStation = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().closeRocketFarHatchTurn1ToFeederStation.get(mStartedLeft));
		closeRocketFeederStationToTurn2 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().closeRocketFeederStationToTurn2.get(mStartedLeft));
		closeRocketTurn2ToCloseHatch = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().closeRocketTurn2ToCloseHatch.get(mStartedLeft));
		closeRocketCloseHatchToBall = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().closeRocketCloseHatchToBall.get(mStartedLeft));
	}

	@Override
	public void done() {
		Drive.getInstance().setVelocity(DriveSignal.NEUTRAL, DriveSignal.NEUTRAL);
	}

	@Override
	protected void routine() throws AutoModeEndedException {
		runAction(highStartToCloseRocketFarHatch);
		runAction(AutomatedActions.placeHatchAction);
		runAction(closeRocketFarHatchToTurn1);
		runAction(new ParallelAction(closeRocketFarHatchTurn1ToFeederStation,
				new SeriesAction(new WaitUntilInsideRegion(new Translation2d(0, -160),
						new Translation2d(50, -110), mStartedLeft),
						AutomatedActions.pickupHatchFeederStation(null))));
		runAction(closeRocketFeederStationToTurn2);
		runAction(closeRocketTurn2ToCloseHatch);
		runAction(AutomatedActions.placeHatchAction);
		runAction(new ParallelAction(closeRocketCloseHatchToBall, AutomatedActions.intakeBallOn((t) -> false)));
	}
}
