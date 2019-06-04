package com.team195.frc2019;

import edu.wpi.first.hal.FRCNetComm;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.NotifierJNI;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class RealtimeRobot extends RobotBase {
	public static final double kDefaultPeriod = 0.02;

	// The C pointer to the notifier object. We don't use it directly, it is
	// just passed to the JNI bindings.
	private final int m_notifier = NotifierJNI.initializeNotifier();

	// The absolute expiration time
	private long m_expirationTime;
	private long m_period;

	private enum Mode {
		kNone,
		kDisabled,
		kAutonomous,
		kTeleop,
		kTest
	}

	private RealtimeRobot.Mode m_lastMode = RealtimeRobot.Mode.kNone;

	protected RealtimeRobot() {
		this(kDefaultPeriod);
	}

	/**
	 * Constructor for RealtimeRobot.
	 *
	 * @param period Period in seconds.
	 */
	protected RealtimeRobot(double period) {
		m_period = (long)(period * 1e6);
		LiveWindow.disableAllTelemetry();
		Shuffleboard.disableActuatorWidgets();
		HAL.report(FRCNetComm.tResourceType.kResourceType_Framework, FRCNetComm.tInstances.kFramework_Timed);
	}

	/**
	 * Provide an alternate "main loop" via startCompetition().
	 */
	@Override
	@SuppressWarnings("UnsafeFinalization")
	public void startCompetition() {
		robotInit();

		// Tell the DS that the robot is ready to be enabled
		HAL.observeUserProgramStarting();

		m_expirationTime = RobotController.getFPGATime() + m_period;
		NotifierJNI.updateNotifierAlarm(m_notifier, m_expirationTime);

		// Loop forever, calling the appropriate mode-dependent function
		while (NotifierJNI.waitForNotifierAlarm(m_notifier) != 0) {
			m_expirationTime += m_period;
			NotifierJNI.updateNotifierAlarm(m_notifier, m_expirationTime);
			// Call the appropriate function depending upon the current robot mode
			if (isDisabled()) {
				// Call DisabledInit() if we are now just entering disabled mode from either a different mode
				// or from power-on.
				if (m_lastMode != RealtimeRobot.Mode.kDisabled) {
					disabledInit();
					m_lastMode = RealtimeRobot.Mode.kDisabled;
				}

				HAL.observeUserProgramDisabled();
				disabledPeriodic();
			} else if (isAutonomous()) {
				// Call AutonomousInit() if we are now just entering autonomous mode from either a different
				// mode or from power-on.
				if (m_lastMode != RealtimeRobot.Mode.kAutonomous) {
					autonomousInit();
					m_lastMode = RealtimeRobot.Mode.kAutonomous;
				}

				HAL.observeUserProgramAutonomous();
				autonomousPeriodic();
			} else if (isOperatorControl()) {
				// Call TeleopInit() if we are now just entering teleop mode from either a different mode or
				// from power-on.
				if (m_lastMode != RealtimeRobot.Mode.kTeleop) {
					teleopInit();
					m_lastMode = RealtimeRobot.Mode.kTeleop;
				}

				HAL.observeUserProgramTeleop();
				teleopPeriodic();
			} else {
				// Call TestInit() if we are now just entering test mode from either a different mode or from
				// power-on.
				if (m_lastMode != RealtimeRobot.Mode.kTest) {
					testInit();
					m_lastMode = RealtimeRobot.Mode.kTest;
				}

				HAL.observeUserProgramTest();
				testPeriodic();
			}

			robotPeriodic();
		}
	}

	/**
	 * Get time period between calls to Periodic() functions.
	 */
	public double getPeriod() {
		return m_period;
	}

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

}
