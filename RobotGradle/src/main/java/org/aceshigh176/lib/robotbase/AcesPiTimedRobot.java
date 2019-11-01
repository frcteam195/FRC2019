/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package org.aceshigh176.lib.robotbase;

import org.aceshigh176.lib.util.UnitUtil;

/**
 * TimedRobot implements the IterativeRobotBase robot program framework.
 *
 * <p>The TimedRobot class is intended to be subclassed by a user creating a robot program.
 *
 * <p>periodic() functions from the base class are called on an interval by a Notifier instance.
 */
public class AcesPiTimedRobot extends AcesPiIterativeRobotBase {
    public static final double kDefaultPeriod = UnitUtil.hzToPeriod(50);

    // The C pointer to the notifier object. We don't use it directly, it is
    // just passed to the JNI bindings.
//    private final int m_notifier = NotifierJNI.initializeNotifier();

    // The absolute expiration time
//    private double m_expirationTime;

    /**
     * Constructor for TimedRobot.
     */
    protected AcesPiTimedRobot() {
        this(kDefaultPeriod);
    }

    /**
     * Constructor for TimedRobot.
     *
     * @param period Period in seconds.
     */
    protected AcesPiTimedRobot(double period) {
        super(period);

//        HAL.report(tResourceType.kResourceType_Framework, tInstances.kFramework_Timed);
    }

//    @Override
//    @SuppressWarnings("NoFinalizer")
//    protected void finalize() {
//        NotifierJNI.stopNotifier(m_notifier);
//        NotifierJNI.cleanNotifier(m_notifier);
//    }

    /**
     * Provide an alternate "main loop" via startCompetition().
     */
    @Override
    @SuppressWarnings("UnsafeFinalization")
    public void startCompetition() {
        robotInit();

        // Tell the DS that the robot is ready to be enabled
//        HAL.observeUserProgramStarting();

//        m_expirationTime = RobotController.getFPGATime() * 1e-6 + m_period;
//        updateAlarm();

        // Loop forever, calling the appropriate mode-dependent function
        while (true) {
//            long curTime = NotifierJNI.waitForNotifierAlarm(m_notifier);
//            if (curTime == 0) {
//                break;
//            }

//            m_expirationTime += m_period;
//            updateAlarm();

            loopFunc();
            // TODO merge this logic with ThreadedNotifier
            try {
                Thread.sleep((int) (m_period * 1000));
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    /**
     * Get time period between calls to Periodic() functions.
     */
    public double getPeriod() {
        return m_period;
    }

    /**
     * Update the alarm hardware to reflect the next alarm.
     */
//    @SuppressWarnings("UnsafeFinalization")
//    private void updateAlarm() {
//        NotifierJNI.updateNotifierAlarm(m_notifier, (long) (m_expirationTime * 1e6));
//    }
}
