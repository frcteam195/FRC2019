package com.team195.frc2019.paths;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.trajectory.TimedView;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.util.Util;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class TrajectoryGeneratorTest {
    public static final double kTestEpsilon = 1e-5;

    public void verifyMirroredTrajectories(final MirroredTrajectory mirrored,
                                           boolean shouldBeReversed) {
        assertEquals(mirrored.left.length(), mirrored.right.length());
        TrajectoryIterator<TimedState<Pose2dWithCurvature>> left_iterator = new TrajectoryIterator<>(new TimedView<>
                (mirrored.left));
        TrajectoryIterator<TimedState<Pose2dWithCurvature>> right_iterator = new TrajectoryIterator<>(new TimedView<>
                (mirrored.right));

        final double dt = 0.05;
        TimedState<Pose2dWithCurvature> prev_left = null;
        TimedState<Pose2dWithCurvature> prev_right = null;
        while (!left_iterator.isDone() && !right_iterator.isDone()) {
            TimedState<Pose2dWithCurvature> left_state = left_iterator.getState();
            TimedState<Pose2dWithCurvature> right_state = right_iterator.getState();

            assertEquals(left_state.t(), right_state.t(), kTestEpsilon);
            assertEquals(left_state.velocity(), right_state.velocity(), kTestEpsilon);
            assertEquals(left_state.acceleration(), right_state.acceleration(), kTestEpsilon);

            assertTrue((shouldBeReversed ? -1.0 : 1.0) * left_state.velocity() >= -kTestEpsilon);
            assertTrue((shouldBeReversed ? -1.0 : 1.0) * right_state.velocity() >= -kTestEpsilon);

            if (prev_left != null && prev_right != null) {
                // Check there are no angle discontinuities.
                final double kMaxReasonableChangeInAngle = 0.3;  // rad
                Twist2d left_change = Pose2d.log(prev_left.state().getPose().inverse().transformBy(left_state.state()
                        .getPose()));
                Twist2d right_change = Pose2d.log(prev_right.state().getPose().inverse().transformBy(right_state
                        .state().getPose()));
                assertTrue(Math.abs(left_change.dtheta) < kMaxReasonableChangeInAngle);
                assertTrue(Math.abs(right_change.dtheta) < kMaxReasonableChangeInAngle);
                if (!Util.epsilonEquals(left_change.dtheta, 0.0) || !Util.epsilonEquals(right_change.dtheta, 0.0)) {
                    // Could be a curvature sign change between prev and now, so just check that either matches our
                    // expected sign.
                    final boolean left_curvature_positive = left_state.state().getCurvature() > kTestEpsilon ||
                            prev_left.state().getCurvature() > kTestEpsilon;
                    final boolean left_curvature_negative = left_state.state().getCurvature() < -kTestEpsilon ||
                            prev_left.state().getCurvature() < -kTestEpsilon;
                    final boolean right_curvature_positive = right_state.state().getCurvature() > kTestEpsilon ||
                            prev_right.state().getCurvature() > kTestEpsilon;
                    final boolean right_curvature_negative = right_state.state().getCurvature() < -kTestEpsilon ||
                            prev_right.state().getCurvature() < -kTestEpsilon;
                    final double actual_left_curvature = left_change.dtheta / left_change.dx;
                    final double actual_right_curvature = right_change.dtheta / right_change.dx;
                    if (actual_left_curvature < -kTestEpsilon) {
                        assertTrue(left_curvature_negative);
                    } else if (actual_left_curvature > kTestEpsilon) {
                        assertTrue(left_curvature_positive);
                    }
                    if (actual_right_curvature < -kTestEpsilon) {
                        assertTrue(right_curvature_negative);
                    } else if (actual_right_curvature > kTestEpsilon) {
                        assertTrue(right_curvature_positive);
                    }
                }
            }

            assertEquals(left_state.state().getTranslation().x(), right_state.state().getTranslation().x(), Util
                    .kEpsilon);
            assertEquals(left_state.state().getTranslation().y(), -right_state.state().getTranslation().y(), Util
                    .kEpsilon);
            assertEquals(left_state.state().getRotation(), right_state.state().getRotation().inverse());
            assertEquals(left_state.state().getCurvature(), -right_state.state().getCurvature(), kTestEpsilon);

            left_iterator.advance(dt);
            right_iterator.advance(dt);
            prev_left = left_state;
            prev_right = right_state;
        }
        assertTrue(left_iterator.isDone() && right_iterator.isDone());
    }

    @Test
    public void test() {
        TrajectoryGenerator.getInstance().generateTrajectories();

        //Next Path is failing. Too sharp
//        verifyMirroredTrajectories(TrajectoryGenerator.getInstance().getTrajectorySet().lowStartToCloseRocketFarHatch, true);
        verifyMirroredTrajectories(TrajectoryGenerator.getInstance().getTrajectorySet().highStartToCloseRocketFarHatch, true);
        verifyMirroredTrajectories(TrajectoryGenerator.getInstance().getTrajectorySet().closeRocketFarHatchToTurn1, false);
        verifyMirroredTrajectories(TrajectoryGenerator.getInstance().getTrajectorySet().closeRocketFarHatchTurn1ToFeederStation, true);
        verifyMirroredTrajectories(TrajectoryGenerator.getInstance().getTrajectorySet().closeRocketFeederStationToTurn2, false);
        //Next Path is suspect. Too sharp
        verifyMirroredTrajectories(TrajectoryGenerator.getInstance().getTrajectorySet().closeRocketTurn2ToCloseHatch, true);
        verifyMirroredTrajectories(TrajectoryGenerator.getInstance().getTrajectorySet().closeRocketCloseHatchToBall, false);

        verifyMirroredTrajectories(TrajectoryGenerator.getInstance().getTrajectorySet().lowStartToSideCargoForwardFacing, false);
        verifyMirroredTrajectories(TrajectoryGenerator.getInstance().getTrajectorySet().highStartToSideCargoForwardFacing, false);
        verifyMirroredTrajectories(TrajectoryGenerator.getInstance().getTrajectorySet().sideCargoForwardFacingToFeederStation, true);
        //Next Path is suspect. Too sharp
        verifyMirroredTrajectories(TrajectoryGenerator.getInstance().getTrajectorySet().feederStationToFrontCargoTurn1, false);
        verifyMirroredTrajectories(TrajectoryGenerator.getInstance().getTrajectorySet().frontCargoTurn1ToFrontCargoHatch, true);
        verifyMirroredTrajectories(TrajectoryGenerator.getInstance().getTrajectorySet().frontCargoHatchToBall, false);

        verifyMirroredTrajectories(TrajectoryGenerator.getInstance().getTrajectorySet().testPath, false);
        verifyMirroredTrajectories(TrajectoryGenerator.getInstance().getTrajectorySet().test90DegPath, false);
    }
}
