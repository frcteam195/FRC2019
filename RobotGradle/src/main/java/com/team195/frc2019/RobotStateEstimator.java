package com.team195.frc2019;

import com.team195.frc2019.loops.Loop;
import com.team195.frc2019.subsystems.Drive;
import com.team195.lib.util.TrajectoryFollowingMotion.PathFollowerRobotState;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Twist2d;

/**
 * Periodically estimates the state of the robot using the robot's distance traveled (compares two waypoints), gyroscope
 * orientation, and velocity, among various other factors. Similar to a car's odometer.
 */
public class RobotStateEstimator implements Loop {
    static RobotStateEstimator instance_ = new RobotStateEstimator();

    public static RobotStateEstimator getInstance() {
        return instance_;
    }

    RobotStateEstimator() {
    }

    PathFollowerRobotState robot_state_ = PathFollowerRobotState.getInstance();
    Drive drive_ = Drive.getInstance();
    double left_encoder_prev_distance_ = 0;
    double right_encoder_prev_distance_ = 0;

    @Override
    public void onFirstStart(double timestamp) {

    }

    @Override
    public synchronized void onStart(double timestamp) {
        left_encoder_prev_distance_ = drive_.getLeftEncoderDistance();
        right_encoder_prev_distance_ = drive_.getRightEncoderDistance();
    }

    @Override
    public synchronized void onLoop(double timestamp) {
        final double left_distance = drive_.getLeftEncoderDistance();
        final double right_distance = drive_.getRightEncoderDistance();
        final Rotation2d gyro_angle = drive_.getHeading();
        final Twist2d odometry_velocity = robot_state_.generateOdometryFromSensors(
                left_distance - left_encoder_prev_distance_, right_distance - right_encoder_prev_distance_, gyro_angle);
        final Twist2d predicted_velocity = Kinematics.forwardKinematics(drive_.getLeftLinearVelocity(),
                drive_.getRightLinearVelocity());
        robot_state_.addObservations(timestamp, odometry_velocity, predicted_velocity);
        left_encoder_prev_distance_ = left_distance;
        right_encoder_prev_distance_ = right_distance;
    }

    @Override
    public void onStop(double timestamp) {
        // no-op
    }

    @Override
    public String getName() {
        return "RobotStateEstimator";
    }

}
