package com.team195.frc2019.auto.actions;

import com.team195.frc2019.subsystems.Drive;

public class OverrideTrajectory extends RunOnceAction {
    @Override
    public void runOnce() {
        Drive.getInstance().overrideTrajectory(true);
    }
}
