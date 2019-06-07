package com.team195.frc2019.subsystems;

import com.team195.frc2019.loops.ILooper;
import com.team195.frc2019.loops.Loop;
import com.team195.frc2019.Kinematics;
import com.team195.frc2019.RobotState;
import com.team195.lib.util.ElapsedTimer;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Twist2d;

import java.util.ArrayList;
import java.util.List;

public class RobotStateEstimator extends Subsystem {
    static RobotStateEstimator instance_ = new RobotStateEstimator();
    private RobotState robot_state_ = RobotState.getInstance();
    private Drive drive_ = Drive.getInstance();

    private final ElapsedTimer loopTimer = new ElapsedTimer();

    private final List<Object> mObjList = new ArrayList<>(5);

    private PeriodicIO mPeriodicIO;

    private RobotStateEstimator() {
        mPeriodicIO = new PeriodicIO();
    }

    public static RobotStateEstimator getInstance() {
        return instance_;
    }

    @Override
    public void stop() {
        // No-op
    }

    @Override
    public void registerEnabledLoops(ILooper looper) {
        looper.register(new EnabledLoop());
    }

    @Override
    public boolean isSystemFaulted() {
        return false;
    }

    @Override
    public boolean runDiagnostics() {
        return true;
    }

    @Override
    public List<Object> generateReport() {
        Pose2d odometry = robot_state_.getLatestFieldToVehicle().getValue();

        mObjList.clear();

        mObjList.add("RobotPoseX");
        mObjList.add(odometry.getTranslation().x());

        mObjList.add("RobotPoseY");
        mObjList.add(odometry.getTranslation().y());

        mObjList.add("RobotPoseTheta");
        mObjList.add(odometry.getRotation().getDegrees());

        mObjList.add("RobotLinearVelocity");
        mObjList.add(robot_state_.getMeasuredVelocity().dx);

        mObjList.add("RobotStateEstimatorLoopTime");
        mObjList.add(mPeriodicIO.robot_state_loop_time);

        return mObjList;
    }

    private class EnabledLoop implements Loop {
        @Override
        public void onFirstStart(double timestamp) {

        }

        @Override
        public synchronized void onStart(double timestamp) {
            mPeriodicIO.left_encoder_prev_distance_ = drive_.getLeftEncoderDistance();
            mPeriodicIO.right_encoder_prev_distance_ = drive_.getRightEncoderDistance();

        }

        @Override
        public synchronized void onLoop(double timestamp) {
            loopTimer.start();
            mPeriodicIO.left_distance = drive_.getLeftEncoderDistance();
            mPeriodicIO.right_distance = drive_.getRightEncoderDistance();
            mPeriodicIO.delta_left = mPeriodicIO.left_distance - mPeriodicIO.left_encoder_prev_distance_;
            mPeriodicIO.delta_right = mPeriodicIO.right_distance - mPeriodicIO.right_encoder_prev_distance_;
            mPeriodicIO.gyro_angle = drive_.getHeading();
            mPeriodicIO.odometry_velocity = robot_state_.generateOdometryFromSensors(
                    mPeriodicIO.delta_left, mPeriodicIO.delta_right, mPeriodicIO.gyro_angle);
            mPeriodicIO.predicted_velocity = Kinematics.forwardKinematics(drive_.getLeftLinearVelocity(),
                    drive_.getRightLinearVelocity());
            robot_state_.addObservations(timestamp, mPeriodicIO.odometry_velocity,
                    mPeriodicIO.predicted_velocity);
            mPeriodicIO.left_encoder_prev_distance_ = mPeriodicIO.left_distance;
            mPeriodicIO.right_encoder_prev_distance_ = mPeriodicIO.right_distance;
            mPeriodicIO.robot_state_loop_time = loopTimer.hasElapsed();
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

    @SuppressWarnings("WeakerAccess")
    public static class PeriodicIO {
        public double left_distance;
        public double right_distance;
        public double delta_left;
        public double delta_right;
        public Rotation2d gyro_angle;
        public Twist2d odometry_velocity;
        public Twist2d predicted_velocity;

        double left_encoder_prev_distance_ = 0.0;
        double right_encoder_prev_distance_ = 0.0;

        // Outputs
        public double robot_state_loop_time;
    }
}

