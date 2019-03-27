package com.team195.frc2019.constants;

import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.subsystems.Turret;
import com.team195.lib.util.RGBColor;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

/**
 * A list of constants used by the rest of the robot code. This include physics constants as well as constants
 * determined through calibrations.
 */
public class Constants {
    public static final double kLooperDt = 0.01;

    public static final int kCANTimeoutMs = 10; //use for on the fly updates
    public static final int kLongCANTimeoutMs = 100; //use for constructors
    public static final int kTalonRetryCount = 3; //use for constructors

    // Control Board
    public static final boolean kUseGamepadForDriving = false;
    public static final boolean kUseGamepadForButtons = true;

    public static final int kDriveGamepadPort = 0;
    public static final int kButtonGamepadPort = 2;
    public static final int kMainThrottleJoystickPort = 0;
    public static final int kMainTurnJoystickPort = 1;
    public static final double kJoystickThreshold = 0.5;
    public static final double kJoystickJogThreshold = 0.4;
    public static final double kJoystickTriggerThreshold = 0.3;


    //Thread prioritization - 5 is default
    public static final int kRobotThreadPriority = 9;
    public static final int kLooperThreadPriority = Thread.MAX_PRIORITY;
    public static final int kCriticalSystemsMonitorThreadPriority = 8;
    public static final int kConnectionMonitorThreadPriority = 7;
    public static final int kLEDThreadPriority = Thread.MIN_PRIORITY;
    public static final int kConsoleReporterThreadPriority = Thread.NORM_PRIORITY;
    public static final int kLogDataReporterThreadPriority = 4;


    public static final String DASHBOARD_IP = "10.1.95.14";
    public static final int LOG_OSC_REPORTER_PORT = 5805;
    public static final int AUTO_SELECTOR_PORT = LOG_OSC_REPORTER_PORT;
    public static final int DASHJOY_RECEIVER_PORT = 5806;


    public static final boolean TUNING_PIDS = true;
    public static final boolean DEBUG = false;
    public static final boolean REPORTING_ENABLED = true;
    public static final boolean REPORT_TO_DRIVERSTATION_INSTEAD_OF_CONSOLE = false;
    public static final RGBColor kDefaultColor = new RGBColor(210, 0, 120);  //Default purple color
    public static final RGBColor kCommLossColor = new RGBColor(255, 0, 0);
    public static final RGBColor kRequestGamePieceColor = new RGBColor(0, 255, 0);
    public static final RGBColor kGotGamePieceColor = kDefaultColor;
    public static final RGBColor kElevatorHomeColor = new RGBColor(0, 0, 255);

    public static final int kActionTimeoutS = 30;

}
