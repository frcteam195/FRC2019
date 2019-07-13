package com.team195.lib.util.statefuser;

import com.team254.lib.geometry.Rotation2d;

public class RotationAdvanced2d extends Rotation2d {
	public static Rotation2d fromQuaternionYaw(Quaternion q) {
		double siny_cosp = 2.0 * (q.getW() * q.getZ() + q.getX() * q.getY());
		double cosy_cosp = 1.0 - 2.0 * (q.getY() * q.getY() + q.getZ() * q.getZ());
		return fromRadians(Math.atan2(siny_cosp, cosy_cosp));
	}

	public static Rotation2d fromQuaternionRoll(Quaternion q) {
		double sinr_cosp = 2.0 * (q.getW() * q.getX() + q.getY() * q.getZ());
		double cosr_cosp = 1.0 - 2.0 * (q.getX() * q.getX() + q.getY() * q.getY());
		return fromRadians(Math.atan2(sinr_cosp, cosr_cosp));
	}

	public static Rotation2d fromQuaternionPitch(Quaternion q) {
		double sinp = 2.0 * (q.getW() * q.getY() - q.getZ() * q.getX());
		if (Math.abs(sinp) >= 1)
			return fromRadians(Math.copySign(Math.PI / 2, sinp)); // use 90 degrees if out of range
		else
			return fromRadians(Math.asin(sinp));
	}
}
