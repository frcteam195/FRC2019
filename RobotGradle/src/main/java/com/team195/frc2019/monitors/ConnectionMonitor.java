package com.team195.frc2019.monitors;

import com.team195.frc2019.Constants;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.reporters.MessageLevel;
import com.team195.frc2019.controllers.LEDController;
import com.team195.lib.util.ThreadRateControl;
import com.team254.lib.util.LatchedBoolean;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/**
 * Keeps track of the robot's connection to the driver station. If it disconnects for more than 1 second, start blinking
 * the LEDs.
 */
public class ConnectionMonitor {

    private static final int MIN_CONNECTION_MONITOR_THREAD_LOOP_MS = 500;
    private static ConnectionMonitor instance = null;
    private boolean hasConnection;
    private boolean runThread = true;
    private double mLastPacketTime;
    private LatchedBoolean mJustReconnected;
    private LatchedBoolean mJustDisconnected;
    private LEDController mLED;
    private final Thread t;

    private ConnectionMonitor() throws Exception {
        ThreadRateControl threadRateControl = new ThreadRateControl();
        mLastPacketTime = 0.0;
        mJustReconnected = new LatchedBoolean();
        mJustDisconnected = new LatchedBoolean();
        runThread = true;
        hasConnection = true;
        mLED = LEDController.getInstance();

        t = new Thread (() -> {
            Thread.currentThread().setName("ConnectionMonitor");
            mLastPacketTime = Timer.getFPGATimestamp();
            threadRateControl.start();

            while (runThread) {
                hasConnection = DriverStation.getInstance().waitForData(1);

                if (hasConnection) {
                    mLastPacketTime = Timer.getFPGATimestamp();
                }
                else {
                    mLED.setLEDColor(Constants.kCommLossColor);
                    mLED.setMessage("sos", true);
                }

                if (mJustReconnected.update(hasConnection))
                    justReconnected();

                if (mJustDisconnected.update(!hasConnection))
                    justDisconnected();

                threadRateControl.doRateControl(MIN_CONNECTION_MONITOR_THREAD_LOOP_MS);
            }
        });
        t.setPriority(Constants.kConnectionMonitorThreadPriority);
        t.start();
    }

    public static ConnectionMonitor getInstance() {
        if(instance == null) {
            try {
                instance = new ConnectionMonitor();
            } catch (Exception ex) {
                ConsoleReporter.report(ex, MessageLevel.DEFCON1);
            }
        }

        return instance;
    }

    public boolean isConnected() {
        return hasConnection;
    }

    public double getLastPacketTime() {
        return mLastPacketTime;
    }

    private void justReconnected() {
        // Reconfigure blink if we are just connected.
        mLED.setLEDColor(Constants.kDefaultColor);
        mLED.configureBlink(LEDController.kDefaultBlinkCount, LEDController.kDefaultBlinkDuration);
        mLED.setRequestedState(LEDController.LEDState.BLINK);
    }

    private void justDisconnected() {
        // Reconfigure blink if we are just disconnected.
        mLED.configureBlink(LEDController.kDefaultBlinkCount, LEDController.kDefaultBlinkDuration * 2.0);
    }

}
