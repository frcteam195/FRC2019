package com.team195.frc2019.auto;

import com.team195.frc2019.auto.actions.Action;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.lib.util.ThreadRateControl;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * An abstract class that is the basis of the robot's autonomous routines. This is implemented in auto modes (which are
 * routines that do actions).
 */
public abstract class AutoModeBase {
    protected double mUpdateRate = 1.0 / 50.0;
    protected boolean mActive = false;
    private ThreadRateControl threadRateControl = new ThreadRateControl();

    protected abstract void routine() throws AutoModeEndedException;

    public void run() {
        mActive = true;

        try {
            routine();
        } catch (AutoModeEndedException e) {
            DriverStation.reportError("AUTO MODE DONE!!!! ENDED EARLY!!!!", false);
            return;
        }

        done();
    }

    public void done() {
        ConsoleReporter.report("Auto mode done");
    }

    public void stop() {
        mActive = false;
    }

    public boolean isActive() {
        return mActive;
    }

    public boolean isActiveWithThrow() throws AutoModeEndedException {
        if (!isActive()) {
            throw new AutoModeEndedException();
        }

        return isActive();
    }

    public void runAction(Action action) throws AutoModeEndedException {
        isActiveWithThrow();
        threadRateControl.start();
        action.start();

        while (isActiveWithThrow() && !action.isFinished()) {
            action.update();
            threadRateControl.doRateControl((int)(mUpdateRate * 1000.0));
        }

        action.done();
    }
}