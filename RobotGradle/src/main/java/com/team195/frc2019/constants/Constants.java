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
    public static final double kLooperDt = 0.010;



    /* ROBOT PHYSICAL CONSTANTS */

    // Wheels
    //public static final double kDriveWheelDiameterInches = 4.875;	//Practice bot calibrated 4.875
    //public static final double kDriveWheelDiameterInches = 5;	//Comp bot measured val
    public static final double kDriveWheelDiameterInches =  5.0 * 0.9625;;
    public static final double kTrackWidthInches = 23.5;
    public static final double kTrackScrubFactor = 1.0; // 0.924 ?

    // Geometry
    public static final double kCenterToFrontBumperDistance = 18.75;
    public static final double kCenterToIntakeDistance = 18.75;
    public static final double kCenterToRearBumperDistance = 18.75;
    public static final double kCenterToSideBumperDistance = 16.375;

    // Path following constants
    public static final double kMinLookAhead = 12.0; // inches
    public static final double kMinLookAheadSpeed = 9.0; // inches per second
    public static final double kMaxLookAhead = 24.0; // inches
    public static final double kMaxLookAheadSpeed = 140.0; // inches per second
    public static final double kDeltaLookAhead = kMaxLookAhead - kMinLookAhead;
    public static final double kDeltaLookAheadSpeed = kMaxLookAheadSpeed - kMinLookAheadSpeed;

    public static final double kInertiaSteeringGain = 0.0; // angular velocity command is multiplied by this gain *
    // our speed
    // in inches per sec
    public static final double kSegmentCompletionTolerance = 5; // inches
    public static final double kPathFollowingMaxAccel = 120.0; // inches per second^2
    public static final double kPathFollowingMaxVel = 120.0; // inches per second

    public static final double kPathFollowingProfileKp = 5.0;   //Used to be 5 when tuning our paths
    public static final double kPathFollowingProfileKi = 0.03;
    public static final double kPathFollowingProfileKv = 0.2;
    public static final double kPathFollowingProfileKffv = 1.0;
    public static final double kPathFollowingProfileKffa = 0.05;
    public static final double kPathFollowingGoalPosTolerance = 1;
    public static final double kPathFollowingGoalVelTolerance = 18.0;
    public static final double kPathStopSteeringDistance = 9.0;



    // Goal tracker constants
    public static final double kMaxGoalTrackAge = 1.0;
    public static final double kMaxTrackerDistance = 18.0;
    public static final double kCameraFrameRate = 30.0;
    public static final double kTrackReportComparatorStablityWeight = 1.0;
    public static final double kTrackReportComparatorAgeWeight = 1.0;

    // Pose of the camera frame w.r.t. the robot frame
    public static final double kCameraXOffset = -3.3211;
    public static final double kCameraYOffset = 0.0;
    public static final double kCameraZOffset = 20.9;
    public static final double kCameraPitchAngleDegrees = 29.56; // Measured on 4/26
    public static final double kCameraYawAngleDegrees = 0.0;
    public static final double kCameraDeadband = 0.0;

    // Target parameters
    // Source of current values: https://firstfrc.blob.core.windows.net/frc2017/Manual/2017FRCGameSeasonManual.pdf
    // Section 3.13
    // ...and https://firstfrc.blob.core.windows.net/frc2017/Drawings/2017FieldComponents.pdf
    // Parts GE-17203-FLAT and GE-17371 (sheet 7)
    public static final double kBoilerTargetTopHeight = 88.0;
    public static final double kBoilerRadius = 7.5;









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
    public static final double kJoystickDeadband = 0.08;


    //Thread prioritization - 5 is default
    public static final int kRobotThreadPriority = 9;
    public static final int kLooperThreadPriority = Thread.MAX_PRIORITY;
    public static final int kConnectionMonitorThreadPriority = 7;
    public static final int kLEDThreadPriority = Thread.MIN_PRIORITY;
    public static final int kConsoleReporterThreadPriority = Thread.NORM_PRIORITY;

    public static final String DASHBOARD_IP = "10.1.95.14";
    public static final int LOG_OSC_REPORTER_PORT = 5805;
    public static final int AUTO_SELECTOR_PORT = LOG_OSC_REPORTER_PORT;
    public static final int DASHJOY_RECEIVER_PORT = 5806;


    public static final boolean TUNING_PIDS = true;
    public static final boolean DEBUG = false;
    public static final boolean LOGGING_ENABLED = true;
    public static final boolean REPORTING_ENABLED = true;
    public static final boolean REPORT_TO_DRIVERSTATION_INSTEAD_OF_CONSOLE = false;

    public static final RGBColor kDefaultColor = new RGBColor(210, 0, 120);  //Default purple color
    public static final RGBColor kCommLossColor = new RGBColor(255, 0, 0);
    public static final RGBColor kRequestGamePieceColor = new RGBColor(0, 255, 0);
    public static final RGBColor kGotGamePieceColor = new RGBColor(0, 0, 255);

    public static final int kActionTimeoutS = 30;

}
