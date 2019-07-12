package com.team195.frc2019.auto.modes;

import com.team195.frc2019.auto.AutoModeEndedException;
import com.team195.frc2019.auto.AutoModeBase;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;

public class DoNothingMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        ConsoleReporter.report("Doing nothing");
    }


    private Pose2d startingPos = new Pose2d(0, 0, Rotation2d.identity());
    @Override
    public Pose2d getStartingCoords() {
        return startingPos;
    }
}
