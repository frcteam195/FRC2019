package com.team195.frc2019.loops;

/**
 * Interface for loops, which are routine that run periodically in the robot code (such as periodic gyroscope
 * calibration, etc.)
 */
public interface Loop {
    public void onFirstStart(double timestamp);

    public void onStart(double timestamp);

    public void onLoop(double timestamp);

    public void onStop(double timestamp);

    public String getName();
}
