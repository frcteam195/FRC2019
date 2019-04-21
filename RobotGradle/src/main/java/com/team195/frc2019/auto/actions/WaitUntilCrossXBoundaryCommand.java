package com.team195.frc2019.auto.actions;

import com.team195.lib.util.TrajectoryFollowingMotion.PathFollowerRobotState;
import edu.wpi.first.wpilibj.Timer;

public class WaitUntilCrossXBoundaryCommand implements Action {

    private double mXBoundary = 0;

    public WaitUntilCrossXBoundaryCommand(double x) {
        mXBoundary = x;
    }

    @Override
    public boolean isFinished() {
        return PathFollowerRobotState.getInstance().getFieldToVehicle(Timer.getFPGATimestamp()).getTranslation().x() > mXBoundary;
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {

    }

    @Override
    public void start() {

    }
}
