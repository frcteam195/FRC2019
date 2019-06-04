package com.team195.frc2019.loops;

import com.team195.frc2019.constants.Constants;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.reporters.MessageLevel;
import com.team195.lib.util.Reportable;
import com.team195.lib.util.TimeoutTimer;
import com.team254.lib.util.CrashTrackingRunnable;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.List;

/**
 * This code runs all of the robot's loops. Loop objects are stored in a List object. They are started when the robot
 * powers up and stopped after the match.
 */
public class Looper implements ILooper, Reportable {
    public final double kPeriod = Constants.kLooperDt;

    private boolean running_;

    private final Notifier notifier_;
    private final List<Loop> loops_;
    private final Object taskRunningLock_ = new Object();
    private double timestamp_ = 0;
    private double now_timestamp_ = 0;
    private double dt_ = 0;
    private String name = "";
    private boolean isFirstStart = true;
    private boolean isFirstRun = true;

    private final CrashTrackingRunnable runnable_;

    public Looper(String name) {
        this.name = name;
        runnable_ = new CrashTrackingRunnable() {
            @Override
            public void runCrashTracked() {
                synchronized (taskRunningLock_) {
                    if (isFirstRun) {
                        Thread.currentThread().setName(name + "Thread");
                        Thread.currentThread().setPriority(Constants.kLooperThreadPriority);
                        isFirstRun = false;
                    }

                    if (running_) {
                        now_timestamp_ = Timer.getFPGATimestamp();
                        try {
                            loops_.forEach((l) -> {
                                l.onLoop(now_timestamp_);
                            });
                        }
                        catch (Exception ex) {
                            ConsoleReporter.report(ex);
                        }
                        dt_ = now_timestamp_ - timestamp_;
                        timestamp_ = now_timestamp_;
                    }
                }
            }
        };

        notifier_ = new Notifier(runnable_);
        running_ = false;
        loops_ = new ArrayList<>();
    }

    @Override
    public synchronized void register(Loop loop) {
        synchronized (taskRunningLock_) {
            loops_.add(loop);
        }
    }

    public synchronized void start() {
        if (!running_) {
//            ConsoleReporter.report("Starting loops");
            synchronized (taskRunningLock_) {
                if (isFirstStart) {
                    timestamp_ = Timer.getFPGATimestamp();
                    try {
                        loops_.forEach((l) -> l.onFirstStart(timestamp_));
                    }
                    catch (Exception ex) {
                        ConsoleReporter.report(ex);
                    }
                }
                timestamp_ = Timer.getFPGATimestamp();
                try {
                    loops_.forEach((l)-> l.onStart(timestamp_));
                }
                catch (Exception ex) {
                    ConsoleReporter.report(ex);
                }
                running_ = true;
                isFirstStart = false;
            }
            notifier_.startPeriodic(kPeriod);
        }
    }

    public synchronized void stop() {
        if (running_) {
//            ConsoleReporter.report("Stopping loops");
            notifier_.stop();
            synchronized (taskRunningLock_) {
                running_ = false;
                timestamp_ = Timer.getFPGATimestamp();
                try {
                    loops_.forEach((l)->l.onStop(timestamp_));
                }
                catch (Exception ex) {
                    ConsoleReporter.report(ex);
                }
            }
        }
    }

    public synchronized void setName(String name) {
        this.name = name;
    }

    private final ArrayList<Object> reportArrayList = new ArrayList<>(2);
    @Override
    public List<Object> generateReport() {
        reportArrayList.clear();
        reportArrayList.add(name+"_dt");
        reportArrayList.add(dt_);
        return reportArrayList;
    }
}
