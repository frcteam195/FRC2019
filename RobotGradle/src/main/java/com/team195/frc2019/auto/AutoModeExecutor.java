package com.team195.frc2019.auto;

import com.team254.lib.util.CrashTrackingRunnable;

/**
 * This class selects, runs, and stops (if necessary) a specified autonomous mode.
 */
public class AutoModeExecutor {
    private AutoModeBase m_auto_mode;
    private Thread m_thread = null;

    public synchronized void setAutoMode(AutoModeBase new_auto_mode) {
        m_auto_mode = new_auto_mode;
        setThread(new Thread(new CrashTrackingRunnable() {
            @Override
            public void runCrashTracked() {
                if (m_auto_mode != null) {
                    m_auto_mode.run();
                }
            }
        }));
    }

    public void start() {
        if (m_thread != null) {
            m_thread.start();
        }
    }

    public void stop() {
        if (m_auto_mode != null) {
            m_auto_mode.stop();
        }

        setThread(null);
    }

    public boolean isRunning() {
        if (m_thread != null) {
           return  m_thread.isAlive();
        }
        return false;
    }

    private void setThread(Thread thread) {
        m_thread = thread;
    }

    public AutoModeBase getAutoMode() {
        return m_auto_mode;
    }
}
