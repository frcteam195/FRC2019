package com.team254.lib.util;

import com.team195.lib.util.FastDoubleToString;

/**
 * A drivetrain command consisting of the left, right motor settings and whether the brake mode is enabled.
 */
public class DriveSignal {
    private double mLeftMotor;
    private double mRightMotor;
    private boolean mBrakeMode;

    public DriveSignal(double left, double right) {
        this(left, right, false);
    }

    public DriveSignal(double left, double right, boolean brakeMode) {
        mLeftMotor = left;
        mRightMotor = right;
        mBrakeMode = brakeMode;
    }

    public static final DriveSignal NEUTRAL = new DriveSignal(0, 0);
    public static final DriveSignal BRAKE = new DriveSignal(0, 0, true);

    public double getLeft() {
        return mLeftMotor;
    }

    public double getRight() {
        return mRightMotor;
    }

    public synchronized void set(double leftMotor, double rightMotor) {
        this.mLeftMotor = leftMotor;
        this.mRightMotor = rightMotor;
    }

    public synchronized void setLeftMotor(double leftMotor) {
        this.mLeftMotor = leftMotor;
    }

    public synchronized void setRightMotor(double rightMotor) {
        this.mRightMotor = rightMotor;
    }

    public boolean getBrakeMode() {
        return mBrakeMode;
    }

    @Override
    public String toString() {
        return "L: " + FastDoubleToString.format(mLeftMotor) + ", R: " + FastDoubleToString.format(mRightMotor) + (mBrakeMode ? ", BRAKE" : "");
    }
}