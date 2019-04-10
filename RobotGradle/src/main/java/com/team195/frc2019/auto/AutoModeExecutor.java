package com.team195.frc2019.auto;

import com.team254.lib.util.CrashTrackingRunnable;

import java.util.concurrent.atomic.AtomicReference;

/**
 * This class selects, runs, and stops (if necessary) a specified autonomous mode.
 */
public class AutoModeExecutor {
    private AtomicReference<AutoModeBase> mAutoMode = new AtomicReference<>(null);
    private AtomicReference<Thread> mThread = new AtomicReference<>(null);

    public void setAutoMode(AutoModeBase new_auto_mode) {
        mAutoMode.set(new_auto_mode);
        mThread.set(new Thread(new CrashTrackingRunnable() {
            @Override
            public void runCrashTracked() {
                if (mAutoMode.get() != null) {
                    mAutoMode.get().run();
                }
            }
        }));
    }

    public void start() {
        if (mThread.get() != null) {
            mThread.get().start();
        }
    }

    public void stop() {
        if (mAutoMode.get() != null) {
            mAutoMode.get().stop();
        }

        mThread.set(null);
    }

    public boolean isRunning() {
        if (mThread.get() != null) {
           return  mThread.get().isAlive();
        }
        return false;
    }

    public AutoModeBase getAutoMode() {
        return mAutoMode.get();
    }
}
