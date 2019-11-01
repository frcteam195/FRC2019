package org.aceshigh176.lib.util;

import java.util.concurrent.atomic.AtomicBoolean;

public class ThreadedNotifier {

    private final Runnable mHandler;
    private final String mName;
    private final int mPriority;

    private final AtomicBoolean mStopRequested;

    private Thread mThread;
    private volatile double mPeriod;

    private double mLastIterationTime;

    public ThreadedNotifier(Runnable handler) {
        this(handler, "Unnamed Threaded Notifier", Thread.NORM_PRIORITY);
    }

    public ThreadedNotifier(Runnable handler, String name, int priority) {
        mHandler = handler;
        mStopRequested = new AtomicBoolean(false);
        mName = name;
        mPriority = priority;
    }

    public void startPeriodic(double period) {
        mPeriod = period;
        mStopRequested.set(false);
        mThread = new Thread(new Runnable() {

            @Override
            public void run() {
                mLastIterationTime = UnitUtil.nanosecondsToSeconds(System.nanoTime());
                while (mStopRequested.get() == false) {
                    double nextIterationTime = mLastIterationTime + mPeriod;
                    mHandler.run();
                    double timeToSleep = 0;
                    while ((timeToSleep = nextIterationTime - UnitUtil.nanosecondsToSeconds(System.nanoTime())) < 0) {
                        nextIterationTime = nextIterationTime + mPeriod;
                    }
                    mLastIterationTime = nextIterationTime;
                    long nsToSleep = (long) UnitUtil.secondsToNanoseconds(timeToSleep);
                    AcesRobotStatus.sleepFor((int) nsToSleep);
                }
            }
        });
        mThread.setDaemon(true);
        mThread.setUncaughtExceptionHandler((thread, error) -> {
            Throwable cause = error.getCause();
            if (cause != null) {
                error = cause;
            }
//            DriverStation.reportError("Unhandled exception: " + error.toString(), error.getStackTrace());
//            DriverStation.reportWarning("Robots should not quit, but yours did!", false);
//            DriverStation.reportError(
//                    "The loopFunc() method (or methods called by it) should have handled "
//                            + "the exception above.", false);
            error.printStackTrace();
            System.exit(1);
        });
        mThread.setName(mName);
        mThread.setPriority(mPriority);
        mThread.start();
    }

    public void stop() {
        mStopRequested.set(true);
    }

    public void setPeriod(double period) {
        mPeriod = period;
    }

}
