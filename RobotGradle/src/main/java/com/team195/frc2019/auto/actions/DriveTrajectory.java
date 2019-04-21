package com.team195.frc2019.auto.actions;

import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.subsystems.Drive;
import com.team195.lib.util.TrajectoryFollowingMotion.Path;
import com.team195.lib.util.TrajectoryFollowingMotion.PathContainer;
import com.team195.lib.util.TrajectoryFollowingMotion.PathFollowerRobotState;
import edu.wpi.first.wpilibj.Timer;

public class DriveTrajectory implements Action {
    private static final Drive mDrive = Drive.getInstance();
    private static final PathFollowerRobotState mRobotState = PathFollowerRobotState.getInstance();

    private PathContainer mPathContainer;
    private Path mPath;
    private final boolean mResetPose;

    public DriveTrajectory(PathContainer p) {
        this(p, false);
    }


    public DriveTrajectory(PathContainer p, boolean resetPose) {
        mPathContainer = p;
        mPath = mPathContainer.buildPath();
        mResetPose = resetPose;
    }

    @Override
    public boolean isFinished() {
        if (mDrive.isDoneWithPath()) {
            ConsoleReporter.report("Trajectory finished");
            return true;
        }
        return false;
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
    }

    @Override
    public void start() {
        ConsoleReporter.report("Starting trajectory!");
        if (mResetPose) {
            mRobotState.reset(Timer.getFPGATimestamp(), mPathContainer.getStartPose());
        }
        mDrive.setWantDrivePath(mPath, mPathContainer.isReversed());
    }
}

