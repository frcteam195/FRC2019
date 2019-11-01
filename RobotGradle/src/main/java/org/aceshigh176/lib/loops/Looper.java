package org.aceshigh176.lib.loops;

import com.team254.lib.util.CrashTracker;
import com.team254.lib.util.CrashTrackingRunnable;
import org.aceshigh176.lib.util.AcesRobotStatus;

import java.util.ArrayList;
import java.util.List;

public abstract class Looper {

    protected double mPeriod;
    private final List<Loopable> mLoops = new ArrayList<>();
    private final List<Loopable> mLoopsToUnregister = new ArrayList<>();

    private final List<LoopableWithStartStop> mLoopsWithStartStopToRegister = new ArrayList<>();
    private final List<LoopableWithStartStop> mLoopsWithStartStop = new ArrayList<>();
    private final List<LoopableWithStartStop> mLoopsWithStartStopToUnregister = new ArrayList<>();

//    private final Telemetry mTelemetry = new Telemetry();

    protected Runnable mRunnable = new CrashTrackingRunnable() {

        @Override
        public void runCrashTracked() {
            synchronized (Looper.this) {
                double timestamp = AcesRobotStatus.getFPGATimestamp();
//                mTelemetry.duration.startTimer();

                // Register currently queued LoopsWithStartStop, then clear the queue
                for (LoopableWithStartStop loopableWithStartStop : mLoopsWithStartStopToRegister) {
                    try {
                        loopableWithStartStop.onStart(timestamp);
                    } catch (Exception e) {
                        e.printStackTrace();
                        CrashTracker.logThrowableCrash(e);
                    }
                }
                mLoopsWithStartStop.addAll(mLoopsWithStartStopToRegister);
                mLoopsWithStartStopToRegister.clear();

                // Execute All loops
                try {
                    for (Loopable loopable : mLoops) {
                        loopable.loop(timestamp);
                    }
                    for (Loopable loopable : mLoopsWithStartStop) {
                        loopable.loop(timestamp);
                    }
                } catch (Exception e) {
                    e.printStackTrace();
                    CrashTracker.logThrowableCrash(e);
                }

                // Unregister currently queued LoopsWithStartStop, then clear the queue
                for (LoopableWithStartStop loopableWithStartStop : mLoopsWithStartStopToUnregister) {
                    try {
                        loopableWithStartStop.onStop(timestamp);
                    } catch (Exception e) {
                        e.printStackTrace();
                        CrashTracker.logThrowableCrash(e);
                    }
                }
                mLoopsWithStartStop.removeAll(mLoopsWithStartStopToUnregister);
                mLoopsWithStartStopToUnregister.clear();

                // Unregister currently queued Loops, then clear the queue
                mLoops.removeAll(mLoopsToUnregister);
                mLoopsToUnregister.clear();

//                mTelemetry.duration.stopTimer();
//                mTelemetry.iteration.mark();
            }
        }
    };

//    @Override
//    public Telemetry getTelemetry() {
//        return mTelemetry;
//    }
//
//    @Override
//    public void outputTelemetry(double timestamp) {
//
//    }
//
//    @Override
//    public void registerTelemetryBuffersTo(WebSocketBufferedGroupDataStorePublisher publisher) {
//
//    }

    public Looper(double period) {
        mPeriod = period;
    }

    public synchronized void register(Loopable loopable) {
        mLoops.add(loopable);
    }

    public synchronized void register(LoopableWithStartStop loopable) {
        mLoopsWithStartStopToRegister.add(loopable);
    }

    /**
     * Queues <pre>loopable</pre> to be unregistered
     *
     * This operation is deferred because loops may need to unregister themselves.
     *
     * @return true if the Loopable is currently registered
     */
    public synchronized boolean queueForUnregister(Loopable loopable) {
        boolean isCurrentlyRegistered = mLoops.contains(loopable);
        if(isCurrentlyRegistered) {
            mLoopsToUnregister.add(loopable);
        }
        return isCurrentlyRegistered;
    }

    public synchronized boolean queueForUnregister(LoopableWithStartStop loopable) {
        boolean isCurrentlyRegistered = mLoopsWithStartStop.contains(loopable);
        if(isCurrentlyRegistered) {
            mLoopsWithStartStopToUnregister.add(loopable);
        }
        return isCurrentlyRegistered;
    }

    public abstract void setPeriod(double period);

//    public class Telemetry extends GroupDataStore {
//        public DataStore_Duration duration = new DataStore_Duration();
//        public DataStore_Iteration iteration = new DataStore_Iteration();
//    }

}
