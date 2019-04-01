package com.team195.frc2019.paths;

import com.team195.frc2019.planners.DriveMotionPlanner;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.CentripetalAccelerationConstraint;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.trajectory.timing.TimingConstraint;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class TrajectoryGenerator {
    private static final double kMaxVelocity = 130.0;
    private static final double kMaxAccel = 130.0;
    private static final double kMaxCentripetalAccelElevatorDown = 110.0;
    private static final double kMaxCentripetalAccel = 100.0;
    private static final double kMaxVoltage = 9.0;
    private static final double kFirstPathMaxVoltage = 9.0;
    private static final double kFirstPathMaxAccel = 130.0;
    private static final double kFirstPathMaxVel = 130.0;

    private static final double kSimpleSwitchMaxAccel = 100.0;
    private static final double kSimpleSwitchMaxCentripetalAccel = 80.0;
    private static final double kSimpleSwitchMaxVelocity = 120.0;

    private static TrajectoryGenerator mInstance = new TrajectoryGenerator();
    private final DriveMotionPlanner mMotionPlanner;
    private TrajectorySet mTrajectorySet = null;

    public static TrajectoryGenerator getInstance() {
        return mInstance;
    }

    private TrajectoryGenerator() {
        mMotionPlanner = new DriveMotionPlanner();
    }

    public void generateTrajectories() {
        if (mTrajectorySet == null) {
            ConsoleReporter.report("Generating trajectories...");
            mTrajectorySet = new TrajectorySet();
            ConsoleReporter.report("Finished trajectory generation");
        }
    }

    public TrajectorySet getTrajectorySet() {
        return mTrajectorySet;
    }

    public MirroredTrajectory generateMirroredTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        return new MirroredTrajectory(mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, max_vel, max_accel, max_voltage));
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, max_vel, max_accel, max_voltage);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double start_vel,  // inches/s
            double end_vel,  // inches/s
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, start_vel, end_vel, max_vel, max_accel, max_voltage);
    }


    public static final Pose2d kLeftRocketPose = new Pose2d(225, 145, Rotation2d.fromDegrees(90));
    public static final Pose2d kRightRocketPose = kLeftRocketPose.mirror();

    public static final Rotation2d backwardsStartRotation = Rotation2d.fromDegrees(180.0);
    public static final Rotation2d forwardsStartRotation = Rotation2d.fromDegrees(0.0);


    // CRITICAL POSES
    // Origin is the center of the robot when the robot is placed against the middle of the alliance station wall.
    // +x is towards the center of the field.
    // +y is to the left.
    // ALL POSES DEFINED FOR THE CASE THAT ROBOT STARTS ON RIGHT! (mirrored about +x axis for LEFT)

    public static final Pose2d kLowStartBackwardsPose = new Pose2d(65, -45, backwardsStartRotation);
    public static final Pose2d kHighStartBackwardsPose = new Pose2d(20, -45, backwardsStartRotation);
    public static final Pose2d kLowStartForwardsPose = new Pose2d(65, -45, forwardsStartRotation);
    public static final Pose2d kHighStartForwardsPose = new Pose2d(20, -45, forwardsStartRotation);

    public static final Pose2d kCloseRocketFarHatchPose = new Pose2d(265, -130, backwardsStartRotation.rotateBy(Rotation2d.fromDegrees(-145.0)));
    public static final Pose2d kCloseRocketCloseHatchPose = new Pose2d(185, -130, backwardsStartRotation.rotateBy(Rotation2d.fromDegrees(-35.0)));
    public static final Pose2d kCloseSideFeederStation = new Pose2d(20, -135, Rotation2d.fromDegrees(0.0));
    public static final Pose2d kCloseSideBallReservoir = new Pose2d(65, -100, Rotation2d.fromDegrees(145.0));

    public static final Pose2d kCloseRocketFarHatchTurn1Point1Pose = kCloseRocketFarHatchPose.transformBy(new Pose2d(20, 60, Rotation2d.fromDegrees(30)));
    public static final Pose2d kCloseRocketFarHatchTurn1Point2Pose = kCloseRocketFarHatchTurn1Point1Pose.transformBy(new Pose2d(-85, -10, Rotation2d.fromDegrees(-75)));

    public static final Pose2d kCloseRocketCloseHatchTurn2Pose = kCloseSideFeederStation.transformBy(new Pose2d(110, 90, Rotation2d.fromDegrees(90)));

    public static final Pose2d kCloseCargoSideHatchForwardFacingPose = new Pose2d(260, -45, Rotation2d.fromDegrees(0));
    public static final Pose2d kCloseCargoFrontHatchPose = new Pose2d(200, -10, backwardsStartRotation);

    public static final Pose2d kCloseCargoFrontHatchTurn1Pose = kCloseSideFeederStation.transformBy(new Pose2d(110, 155, Rotation2d.fromDegrees(90)));

    public class TrajectorySet {

        public final MirroredTrajectory lowStartToCloseRocketFarHatch;
        public final MirroredTrajectory highStartToCloseRocketFarHatch;
        public final MirroredTrajectory closeRocketFarHatchToTurn1;
        public final MirroredTrajectory closeRocketFarHatchTurn1ToFeederStation;
        public final MirroredTrajectory closeRocketFeederStationToTurn2;
        public final MirroredTrajectory closeRocketTurn2ToCloseHatch;
        public final MirroredTrajectory closeRocketCloseHatchToBall;

        public final MirroredTrajectory lowStartToSideCargoForwardFacing;
        public final MirroredTrajectory highStartToSideCargoForwardFacing;
        public final MirroredTrajectory sideCargoForwardFacingToFeederStation;
        public final MirroredTrajectory feederStationToFrontCargoTurn1;
        public final MirroredTrajectory frontCargoTurn1ToFrontCargoHatch;
        public final MirroredTrajectory frontCargoHatchToBall;

        private TrajectorySet() {
            lowStartToCloseRocketFarHatch = generateMirroredTrajectory(true, Arrays.asList(
                    kLowStartBackwardsPose,
                    kCloseRocketFarHatchPose),
                    Collections.singletonList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kFirstPathMaxVel,
                    kFirstPathMaxAccel,
                    kFirstPathMaxVoltage);

            highStartToCloseRocketFarHatch = generateMirroredTrajectory(true, Arrays.asList(
                    kHighStartBackwardsPose,
                    kCloseRocketFarHatchPose),
                    Collections.singletonList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kFirstPathMaxVel,
                    kFirstPathMaxAccel,
                    kFirstPathMaxVoltage);

            closeRocketFarHatchToTurn1 = generateMirroredTrajectory(false, Arrays.asList(
                    kCloseRocketFarHatchPose,
                    kCloseRocketFarHatchTurn1Point1Pose),
                    Collections.singletonList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity,
                    kMaxAccel,
                    kMaxVoltage);

            closeRocketFarHatchTurn1ToFeederStation = generateMirroredTrajectory(true, Arrays.asList(
                    kCloseRocketFarHatchTurn1Point1Pose,
                    kCloseRocketFarHatchTurn1Point2Pose,
                    kCloseSideFeederStation),
                    Collections.singletonList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity,
                    kMaxAccel,
                    kMaxVoltage);

            closeRocketFeederStationToTurn2 = generateMirroredTrajectory(false, Arrays.asList(
                    kCloseSideFeederStation,
                    kCloseRocketCloseHatchTurn2Pose),
                    Collections.singletonList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity,
                    kMaxAccel,
                    kMaxVoltage);

            closeRocketTurn2ToCloseHatch = generateMirroredTrajectory(true, Arrays.asList(
                    kCloseRocketCloseHatchTurn2Pose,
                    kCloseRocketCloseHatchPose),
                    Collections.singletonList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity,
                    kMaxAccel,
                    kMaxVoltage);

            closeRocketCloseHatchToBall = generateMirroredTrajectory(false, Arrays.asList(
                    kCloseRocketCloseHatchPose,
                    kCloseSideBallReservoir),
                    Collections.singletonList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity,
                    kMaxAccel,
                    kMaxVoltage);

            lowStartToSideCargoForwardFacing = generateMirroredTrajectory(false, Arrays.asList(
                    kLowStartForwardsPose,
                    kCloseCargoSideHatchForwardFacingPose),
                    Collections.singletonList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kFirstPathMaxVel,
                    kFirstPathMaxAccel,
                    kFirstPathMaxVoltage);

            highStartToSideCargoForwardFacing = generateMirroredTrajectory(false, Arrays.asList(
                    kHighStartForwardsPose,
                    kCloseCargoSideHatchForwardFacingPose),
                    Collections.singletonList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kFirstPathMaxVel,
                    kFirstPathMaxAccel,
                    kFirstPathMaxVoltage);

            sideCargoForwardFacingToFeederStation = generateMirroredTrajectory(true, Arrays.asList(
                    kCloseCargoSideHatchForwardFacingPose,
                    kCloseSideFeederStation),
                    Collections.singletonList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity,
                    kMaxAccel,
                    kMaxVoltage);

            feederStationToFrontCargoTurn1 = generateMirroredTrajectory(false, Arrays.asList(
                    kCloseSideFeederStation,
                    kCloseCargoFrontHatchTurn1Pose),
                    Collections.singletonList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity,
                    kMaxAccel,
                    kMaxVoltage);

            frontCargoTurn1ToFrontCargoHatch = generateMirroredTrajectory(true, Arrays.asList(
                    kCloseCargoFrontHatchTurn1Pose,
                    kCloseCargoFrontHatchPose),
                    Collections.singletonList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity,
                    kMaxAccel,
                    kMaxVoltage);

            frontCargoHatchToBall = generateMirroredTrajectory(false, Arrays.asList(
                    kCloseCargoFrontHatchPose,
                    kCloseSideBallReservoir),
                    Collections.singletonList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity,
                    kMaxAccel,
                    kMaxVoltage);
        }

    }
}
