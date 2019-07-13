package com.team195.frc2019.auto.modes;

import com.team195.frc2019.auto.AutoModeEndedException;
import com.team195.frc2019.auto.AutoModeBase;
import com.team195.frc2019.auto.actions.DriveTrajectory;
import com.team195.frc2019.paths.TrajectoryGenerator;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;

public class TestMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        ConsoleReporter.report("Test mode");
//        Drive.getInstance().startLogging();

        runAction(new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().test90DegPath.get(false), true));

        ConsoleReporter.report("Test mode done!");
//        Drive.getInstance().stopLogging();

        /*runAction(new DriveTrajectory(TrajectoryGenerator.getInstance().generateTrajectory(
                false,
                Arrays.asList(Pose2d.identity(), Pose2d.fromTranslation(new Translation2d(48.0, 0.0)),
                        new Pose2d(new Translation2d(96.0, -48.0), Rotation2d.fromDegrees(-90.0)),
                        new Pose2d(new Translation2d(96.0, -96.0), Rotation2d.fromDegrees(-90.0))),
                Arrays.asList(new CentripetalAccelerationConstraint(80.0)),
                120.0, 120.0, 10.0),true));*/

/*
        runAction(new DriveTrajectory(TrajectoryGenerator.getInstance().generateTrajectory(
                false,
                Arrays.asList(
                        new Pose2d(new Translation2d(96.0, -96.0), Rotation2d.fromDegrees(90.0)),
                        new Pose2d(new Translation2d(96.0, -48.0), Rotation2d.fromDegrees(90.0)),
                        Pose2d.fromRotation(Rotation2d.fromDegrees(180.0))),
                Arrays.asList(new CentripetalAccelerationConstraint(120.0)),
                120.0, 120.0, 10.0)));*/
    }

    @Override
    public Pose2d getStartingCoords() {
        return new Pose2d(0, 0, Rotation2d.identity());
    }
}
