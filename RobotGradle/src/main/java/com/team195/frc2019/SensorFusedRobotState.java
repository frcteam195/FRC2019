package com.team195.frc2019;

import com.team195.frc2019.subsystems.Drive;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;

import java.util.Map;

public class SensorFusedRobotState {
	private static SensorFusedRobotState instance_ = new SensorFusedRobotState();

	public static SensorFusedRobotState getInstance() {
		return instance_;
	}

	private static final int kObservationBufferSize = 100;

	// FPGATimestamp -> Pose2d or Rotation2d
	private final InterpolatingTreeMap<InterpolatingDouble, Pose2d> field_to_vehicle_ = new InterpolatingTreeMap<>(kObservationBufferSize);

	private SensorFusedRobotState() {
		reset(0, new Pose2d());
	}

	/**
	 * Resets the field to robot transform (robot's position on the field)
	 */
	public void reset(double start_time, Pose2d initial_field_to_vehicle) {
		field_to_vehicle_.clear();
		field_to_vehicle_.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
		Drive.getInstance().setHeading(initial_field_to_vehicle.getRotation());
	}

	/**
	 * Returns the robot's position on the field at a certain time. Linearly interpolates between stored robot positions
	 * to fill in the gaps.
	 */
	public Pose2d getFieldToVehicle(double timestamp) {
		return field_to_vehicle_.getInterpolated(new InterpolatingDouble(timestamp));
	}

	public Map.Entry<InterpolatingDouble, Pose2d> getLatestFieldToVehicle() {
		return field_to_vehicle_.lastEntry();
	}

	public void addFieldToVehicleObservation(double timestamp, Pose2d observation) {
		field_to_vehicle_.put(new InterpolatingDouble(timestamp), observation);
	}
}
