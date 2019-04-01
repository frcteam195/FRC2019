package com.team195.frc2019;

import com.team195.frc2019.paths.TrajectoryGenerator;
import org.junit.jupiter.api.Test;

public class AutoModeTiming {
    TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    boolean mStartedLeft = true;

    @Test
    void checkTiming() {
        mTrajectoryGenerator.generateTrajectories();

        double rocketHatchDuration = mTrajectoryGenerator.getTrajectorySet().lowStartToCloseRocketFarHatch.get(mStartedLeft).getLastState().t() +
//                mTrajectoryGenerator.getTrajectorySet().highStartToCloseRocketFarHatch.get(mStartedLeft).getLastState().t() +
                mTrajectoryGenerator.getTrajectorySet().closeRocketFarHatchToTurn1.get(mStartedLeft).getLastState().t() +
                mTrajectoryGenerator.getTrajectorySet().closeRocketFarHatchTurn1ToFeederStation.get(mStartedLeft).getLastState().t() +
                mTrajectoryGenerator.getTrajectorySet().closeRocketFeederStationToTurn2.get(mStartedLeft).getLastState().t() +
                mTrajectoryGenerator.getTrajectorySet().closeRocketTurn2ToCloseHatch.get(mStartedLeft).getLastState().t() +
                mTrajectoryGenerator.getTrajectorySet().closeRocketCloseHatchToBall.get(mStartedLeft).getLastState().t();
//        double nearScaleBestCaseWait = 3 * AutoConstants.kWaitForCubeTime;
//        double nearScaleWorstCaseWait = nearScaleBestCaseWait + 0.5 * 4;


        double cargoHatchDuration = mTrajectoryGenerator.getTrajectorySet().lowStartToSideCargoForwardFacing.get(mStartedLeft).getLastState().t() +
//                mTrajectoryGenerator.getTrajectorySet().highStartToSideCargoForwardFacing.get(mStartedLeft).getLastState().t() +
                mTrajectoryGenerator.getTrajectorySet().sideCargoForwardFacingToFeederStation.get(mStartedLeft).getLastState().t() +
                mTrajectoryGenerator.getTrajectorySet().feederStationToFrontCargoTurn1.get(mStartedLeft).getLastState().t() +
                mTrajectoryGenerator.getTrajectorySet().frontCargoTurn1ToFrontCargoHatch.get(mStartedLeft).getLastState().t() +
                mTrajectoryGenerator.getTrajectorySet().frontCargoHatchToBall.get(mStartedLeft).getLastState().t();
//        double farScaleBestCaseWait = 2 * AutoConstants.kWaitForCubeTime;
//        double farScaleWorstCaseWait = farScaleBestCaseWait + 0.5 * 3;

        System.out.println("Rocket Hatch Only:");
        System.out.println("\tTrajectory Duration: " + rocketHatchDuration);
//        System.out.println("\tBest Case Wait Duration: " + nearScaleBestCaseWait);
//        System.out.println("\tWorst Case Wait Duration: " + nearScaleWorstCaseWait);
        System.out.println("Cargo Hatch Only:");
        System.out.println("\tTrajectory Duration: " + cargoHatchDuration);
//        System.out.println("\tBest Case Wait Duration: " + farScaleBestCaseWait);
//        System.out.println("\tWorst Case Wait Duration: " + farScaleWorstCaseWait);
    }
}
