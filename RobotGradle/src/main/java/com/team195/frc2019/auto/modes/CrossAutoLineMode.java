package com.team195.frc2019.auto.modes;

import com.team195.frc2019.auto.AutoModeEndedException;
import com.team195.frc2019.auto.AutoModeBase;
import com.team195.frc2019.auto.actions.OpenLoopDrive;
import com.team195.frc2019.auto.actions.WaitAction;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;

public class CrossAutoLineMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        ConsoleReporter.report("Running Cross auto line");
        runAction(new WaitAction(5.0));
        runAction(new OpenLoopDrive(-0.3, -0.3, 5.0));
    }

    @Override
    public Pose2d getStartingCoords() {
        return new Pose2d(0, 0, Rotation2d.identity());
    }
}
