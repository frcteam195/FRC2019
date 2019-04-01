package com.team195.frc2019.paths;

import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryUtil;
import com.team254.lib.trajectory.timing.TimedState;

public class MirroredTrajectory {
	public MirroredTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> right) {
		this.right = right;
		this.left = TrajectoryUtil.mirrorTimed(right);
	}

	public Trajectory<TimedState<Pose2dWithCurvature>> get(boolean left) {
		return left ? this.left : this.right;
	}

	public final Trajectory<TimedState<Pose2dWithCurvature>> left;
	public final Trajectory<TimedState<Pose2dWithCurvature>> right;
}