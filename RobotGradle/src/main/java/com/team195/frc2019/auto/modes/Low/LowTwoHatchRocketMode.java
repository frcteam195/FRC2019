package com.team195.frc2019.auto.modes.Low;

import com.team195.frc2019.auto.AutoModeBase;
import com.team195.frc2019.auto.AutoModeEndedException;
import com.team195.frc2019.auto.actions.DriveTrajectory;
import com.team195.frc2019.auto.actions.ParallelAction;
import com.team195.frc2019.auto.actions.SeriesAction;
import com.team195.frc2019.auto.actions.WaitUntilInsideRegion;
import com.team195.frc2019.auto.autonomy.AutomatedAction;
import com.team195.frc2019.auto.autonomy.AutomatedActions;
import com.team195.frc2019.paths.TrajectoryGenerator;
import com.team195.frc2019.subsystems.Drive;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.DriveSignal;

public class LowTwoHatchRocketMode extends AutoModeBase {
	private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();

	private final DriveTrajectory lowStartToCloseRocketFarHatch;
	private final DriveTrajectory closeRocketFarHatchToTurn1;
	private final DriveTrajectory closeRocketFarHatchTurn1ToFeederStation;
	private final DriveTrajectory closeRocketFeederStationToTurn2;
	private final DriveTrajectory closeRocketTurn2ToCloseHatch;
	private final DriveTrajectory closeRocketCloseHatchToBall;

	private final boolean mStartedLeft;

	public LowTwoHatchRocketMode(boolean robotStartedOnLeft) {
		mStartedLeft = robotStartedOnLeft;

		lowStartToCloseRocketFarHatch = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().lowStartToCloseRocketFarHatch.get(mStartedLeft), true);
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
		runAction(lowStartToCloseRocketFarHatch);
		runAction(AutomatedActions.placeHatch());
		runAction(closeRocketFarHatchToTurn1);
//		runAction(new ParallelAction(closeRocketFarHatchTurn1ToFeederStation,
//				new SeriesAction(new WaitUntilInsideRegion(new Translation2d(0, -160),
//						new Translation2d(50, -110), mStartedLeft),
//						AutomatedActions.pickupHatchFeederStation())));
//		runAction(closeRocketFeederStationToTurn2);
//		runAction(closeRocketTurn2ToCloseHatch);
//		runAction(AutomatedActions.placeHatch());
//		runAction(new ParallelAction(closeRocketCloseHatchToBall, AutomatedActions.intakeBallOn((t) -> false)));
	}
}
