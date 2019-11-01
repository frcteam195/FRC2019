/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.aceshigh176.lib.robotbase;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/**
 * IterativeRobotBase implements a specific type of robot program framework, extending the RobotBase
 * class.
 *
 * <p>The IterativeRobotBase class does not implement startCompetition(), so it should not be used
 * by teams directly.
 *
 * <p>This class provides the following functions which are called by the main loop,
 * startCompetition(), at the appropriate times:
 *
 * <p>robotInit() -- provide for initialization at robot power-on
 *
 * <p>init() functions -- each of the following functions is called once when the
 * appropriate mode is entered:
 *   - disabledInit()   -- called each and every time disabled is entered from
 *                         another mode
 *   - autonomousInit() -- called each and every time autonomous is entered from
 *                         another mode
 *   - teleopInit()     -- called each and every time teleop is entered from
 *                         another mode
 *   - testInit()       -- called each and every time test is entered from
 *                         another mode
 *
 * <p>periodic() functions -- each of these functions is called on an interval:
 *   - robotPeriodic()
 *   - disabledPeriodic()
 *   - autonomousPeriodic()
 *   - teleopPeriodic()
 *   - testPeriodic()
 */
@SuppressWarnings("PMD.TooManyMethods")
public abstract class AcesPiIterativeRobotBase extends AcesPiRobotBase {
//    private final Logger log = LogManager.getLogger(AcesPiIterativeRobotBase.class);

    protected double m_period;

    private RobotOperationalMode mLastMode = RobotOperationalMode.kDisabled;
//    private final Watchdog m_watchdog;
//    private DataStore_Duration mWatchdogDuration = new DataStore_Duration();
//
//    public class AcesPiIterativeRobotData {
//        public final DataStore_Iteration mainRobotLoopIteration = new DataStore_Iteration();
//        public final DataStore_Duration mainRobotLoopDuration = new DataStore_Duration();
//    }
//
//    private final AcesPiIterativeRobotData mData = new AcesPiIterativeRobotData();
//
//    public class AcesPiIterativeRobotDataEnhanced {
//        public final DataStore_Duration disabledInitDuration = new DataStore_Duration();
//        public final DataStore_Duration disabledPeriodicDuration = new DataStore_Duration();
//        public final DataStore_Duration teleopInitDuration = new DataStore_Duration();
//        public final DataStore_Duration teleopPeriodicDuration = new DataStore_Duration();
//        public final DataStore_Duration autoInitDuration = new DataStore_Duration();
//        public final DataStore_Duration autoPeriodicDuration = new DataStore_Duration();
//        public final DataStore_Duration testInitDuration = new DataStore_Duration();
//        public final DataStore_Duration testPeriodicDuration = new DataStore_Duration();
//
//        public final DataStore_Duration robotPeriodicDuration = new DataStore_Duration();
//    }
//
//    private final AcesPiIterativeRobotDataEnhanced mDataEnhanced = new AcesPiIterativeRobotDataEnhanced();

    /**
     * Constructor for IterativeRobotBase.
     *
     * @param period Period in seconds.
     */
    protected AcesPiIterativeRobotBase(double period) {
        m_period = period;
//        m_watchdog = new Watchdog(period, this::printLoopOverrunMessage);
    }

    /**
     * Provide an alternate "main loop" via startCompetition().
     */
    @Override
    public abstract void startCompetition();

    /* ----------- Overridable initialization code ----------------- */

    /**
     * Robot-wide initialization code should go here.
     *
     * <p>Users should override this method for default Robot-wide initialization which will be called
     * when the robot is first powered on. It will be called exactly one time.
     *
     * <p>Warning: the Driver Station "Robot Code" light and FMS "Robot Ready" indicators will be off
     * until RobotInit() exits. Code in RobotInit() that waits for enable will cause the robot to
     * never indicate that the code is ready, causing the robot to be bypassed in a match.
     */
    public void robotInit() {
        System.out.println("Default robotInit() method... Override me!");
    }

    /**
     * Initialization code for disabled mode should go here.
     *
     * <p>Users should override this method for initialization code which will be called each time the
     * robot enters disabled mode.
     */
    public void disabledInit() {
        System.out.println("Default disabledInit() method... Override me!");
    }

    /**
     * Initialization code for autonomous mode should go here.
     *
     * <p>Users should override this method for initialization code which will be called each time the
     * robot enters autonomous mode.
     */
    public void autonomousInit() {
        System.out.println("Default autonomousInit() method... Override me!");
    }

    /**
     * Initialization code for teleop mode should go here.
     *
     * <p>Users should override this method for initialization code which will be called each time the
     * robot enters teleop mode.
     */
    public void teleopInit() {
        System.out.println("Default teleopInit() method... Override me!");
    }

    /**
     * Initialization code for test mode should go here.
     *
     * <p>Users should override this method for initialization code which will be called each time the
     * robot enters test mode.
     */
    @SuppressWarnings("PMD.JUnit4TestShouldUseTestAnnotation")
    public void testInit() {
        System.out.println("Default testInit() method... Override me!");
    }

    /* ----------- Overridable periodic code ----------------- */

    private boolean m_rpFirstRun = true;

    /**
     * Periodic code for all robot modes should go here.
     */
    public void robotPeriodic() {
        if (m_rpFirstRun) {
            System.out.println("Default robotPeriodic() method... Override me!");
            m_rpFirstRun = false;
        }
    }

    private boolean m_dpFirstRun = true;

    /**
     * Periodic code for disabled mode should go here.
     */
    public void disabledPeriodic() {
        if (m_dpFirstRun) {
            System.out.println("Default disabledPeriodic() method... Override me!");
            m_dpFirstRun = false;
        }
    }

    private boolean m_apFirstRun = true;

    /**
     * Periodic code for autonomous mode should go here.
     */
    public void autonomousPeriodic() {
        if (m_apFirstRun) {
            System.out.println("Default autonomousPeriodic() method... Override me!");
            m_apFirstRun = false;
        }
    }

    private boolean m_tpFirstRun = true;

    /**
     * Periodic code for teleop mode should go here.
     */
    public void teleopPeriodic() {
        if (m_tpFirstRun) {
            System.out.println("Default teleopPeriodic() method... Override me!");
            m_tpFirstRun = false;
        }
    }

    private boolean m_tmpFirstRun = true;

    /**
     * Periodic code for test mode should go here.
     */
    @SuppressWarnings("PMD.JUnit4TestShouldUseTestAnnotation")
    public void testPeriodic() {
        if (m_tmpFirstRun) {
            System.out.println("Default testPeriodic() method... Override me!");
            m_tmpFirstRun = false;
        }
    }

    protected void loopFunc() {
//        m_watchdog.reset();
//        mWatchdogDuration.startTimer();
//        mData.mainRobotLoopIteration.mark();
//        mData.mainRobotLoopDuration.startTimer();

        // Call the appropriate function depending upon the current robot mode
        if (isDisabled()) {
            // Call DisabledInit() if we are now just entering disabled mode from either a different mode
            // or from power-on.
            if (mLastMode != RobotOperationalMode.kDisabled) {
//                LiveWindow.setEnabled(false);
//                Shuffleboard.disableActuatorWidgets();
//                mDataEnhanced.disabledInitDuration.startTimer();
                disabledInit();
//                mDataEnhanced.disabledInitDuration.stopTimer();
//                m_watchdog.addEpoch("disabledInit()");
                mLastMode = RobotOperationalMode.kDisabled;
            }

//            HAL.observeUserProgramDisabled();
//            mDataEnhanced.disabledPeriodicDuration.startTimer();
            disabledPeriodic();
//            mDataEnhanced.disabledPeriodicDuration.stopTimer();
//            m_watchdog.addEpoch("disablePeriodic()");
        } else if (isAutonomous()) {
            // Call AutonomousInit() if we are now just entering autonomous mode from either a different
            // mode or from power-on.
            if (mLastMode != RobotOperationalMode.kAutonomous) {
//                LiveWindow.setEnabled(false);
//                Shuffleboard.disableActuatorWidgets();
//                mDataEnhanced.autoInitDuration.startTimer();
                autonomousInit();
//                mDataEnhanced.autoInitDuration.stopTimer();
//                m_watchdog.addEpoch("autonomousInit()");
                mLastMode = RobotOperationalMode.kAutonomous;
            }

//            HAL.observeUserProgramAutonomous();
//            mDataEnhanced.autoPeriodicDuration.startTimer();
            autonomousPeriodic();
//            mDataEnhanced.autoPeriodicDuration.stopTimer();
//            m_watchdog.addEpoch("autonomousPeriodic()");
        } else if (isOperatorControl()) {
            // Call TeleopInit() if we are now just entering teleop mode from either a different mode or
            // from power-on.
            if (mLastMode != RobotOperationalMode.kTeleop) {
//                LiveWindow.setEnabled(false);
//                Shuffleboard.disableActuatorWidgets();
//                mDataEnhanced.teleopInitDuration.startTimer();
                teleopInit();
//                mDataEnhanced.teleopInitDuration.stopTimer();
//                m_watchdog.addEpoch("teleopInit()");
                mLastMode = RobotOperationalMode.kTeleop;
            }

//            HAL.observeUserProgramTeleop();
//            mDataEnhanced.teleopPeriodicDuration.startTimer();
            teleopPeriodic();
//            mDataEnhanced.teleopPeriodicDuration.stopTimer();
//            m_watchdog.addEpoch("teleopPeriodic()");
        } else {
            // Call TestInit() if we are now just entering test mode from either a different mode or from
            // power-on.
            if (mLastMode != RobotOperationalMode.kTest) {
//                LiveWindow.setEnabled(true);
//                Shuffleboard.enableActuatorWidgets();
//                mDataEnhanced.testInitDuration.startTimer();
                testInit();
//                mDataEnhanced.testInitDuration.stopTimer();
//                m_watchdog.addEpoch("testInit()");
                mLastMode = RobotOperationalMode.kTest;
            }

//            HAL.observeUserProgramTest();
//            mDataEnhanced.testPeriodicDuration.startTimer();
            testPeriodic();
//            mDataEnhanced.testPeriodicDuration.stopTimer();
//            m_watchdog.addEpoch("testPeriodic()");
        }

//        mDataEnhanced.robotPeriodicDuration.startTimer();
        robotPeriodic();
//        mDataEnhanced.robotPeriodicDuration.stopTimer();
//        m_watchdog.addEpoch("robotPeriodic()");
//        m_watchdog.disable();
//        SmartDashboard.updateValues();

//        LiveWindow.updateValues();
        Shuffleboard.update();

        // Warn on loop time overruns
//        if (m_watchdog.isExpired()) {
//            m_watchdog.printEpochs();
//        }

//        mData.mainRobotLoopDuration.stopTimer();
//        mWatchdogDuration.stopTimer();
//        if(mWatchdogDuration.get() > .02) { // TODO refactor this to a constant
//            log.error("Loop overrun of {}s, mode={}", mWatchdogDuration.get(), mLastMode);
//        }
    }
}
