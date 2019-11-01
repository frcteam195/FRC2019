package org.aceshigh176.lib.loops;

import org.aceshigh176.lib.util.ThreadedNotifier;

public class ThreadedLooper extends Looper {

    ThreadedNotifier mNotifier;

    public ThreadedLooper(double period) {
        this(period, "Unnamed Threaded Looper", Thread.NORM_PRIORITY);
    }

    public ThreadedLooper(double period, String name, int priority) {
        super(period);
        mNotifier = new ThreadedNotifier(mRunnable, name, priority);
        mNotifier.startPeriodic(mPeriod);
    }

    public void setPeriod(double period) {
        mNotifier.setPeriod(period);
    }

}
