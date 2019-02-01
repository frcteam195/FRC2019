package com.team195.frc2019;

import com.team195.lib.util.RGBColor;
import edu.wpi.first.wpilibj.Solenoid;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

/**
 * A list of constants used by the rest of the robot code. This include physics constants as well as constants
 * determined through calibrations.
 */
public class Constants {
    public static final double kLooperDt = 0.01;

    /* ROBOT PHYSICAL CONSTANTS */

    // Wheels
    public static final double kDriveWheelTrackWidthInches = 25.54;
    public static final double kDriveWheelDiameterInches = 3.92820959548 * 0.99;
    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
    public static final double kTrackScrubFactor = 1.0;  // Tune me!

    // Tuned dynamics
    public static final double kRobotLinearInertia = 60.0;  // kg TODO tune
    public static final double kRobotAngularInertia = 10.0;  // kg m^2 TODO tune
    public static final double kRobotAngularDrag = 12.0;  // N*m / (rad/sec) TODO tune
    public static final double kDriveVIntercept = 1.055;  // V
    public static final double kDriveKv = 0.135;  // V per rad/s
    public static final double kDriveKa = 0.012;  // V per rad/s^2

    // Geometry
    public static final double kCenterToFrontBumperDistance = 38.25 / 2.0;
    public static final double kCenterToRearBumperDistance = 38.25 / 2.0;
    public static final double kCenterToSideBumperDistance = 33.75 / 2.0;

    // Pose of the LIDAR frame w.r.t. the robot frame
    // TODO measure in CAD/on robot!
    public static final double kLidarXOffset = -3.3211;
    public static final double kLidarYOffset = 0.0;
    public static final double kLidarYawAngleDegrees = 0.0;

    /* CONTROL LOOP GAINS */

    // Gearing and mechanical constants.
    public static final double kDriveDownShiftVelocity = 9.5 * 12.0;  // inches per second
    public static final double kDriveDownShiftAngularVelocity = Math.PI / 2.0; // rad/sec
    public static final double kDriveUpShiftVelocity = 11.0 * 12.0;  // inches per second

    public static final double kPathKX = 4.0;  // units/s per unit of error
    public static final double kPathLookaheadTime = 0.4;  // seconds to look ahead along the path for steering
    public static final double kPathMinLookaheadDistance = 24.0;  // inches

    // PID gains for drive velocity loop (LOW GEAR)
    // Units: setpoint, error, and output are in ticks per second.
    public static final double kDriveLowGearVelocityKp = 0.9;
    public static final double kDriveLowGearVelocityKi = 0.0;
    public static final double kDriveLowGearVelocityKd = 10.0;
    public static final double kDriveLowGearVelocityKf = 0.0;
    public static final int kDriveLowGearVelocityIZone = 0;
    public static final double kDriveVoltageRampRate = 0.0;

    // Do not change anything after this line unless you rewire the robot and
    // update the spreadsheet!
    // Port assignments should match up with the spreadsheet here:
    // https://docs.google.com/spreadsheets/d/179YszqnEWPWInuHUrYJnYL48LUL7LUhZrnvmNu1kujE/edit#gid=0

    /* I/O */
    // (Note that if multiple talons are dedicated to a mechanism, any sensors
    // are attached to the master)

    public static final int kCANTimeoutMs = 10; //use for on the fly updates
    public static final int kLongCANTimeoutMs = 100; //use for constructors
    public static final int kTalonRetryCount = 3; //use for constructors


    // Drive
    public static final int kLeftDriveMasterId =4;
    public static final int kLeftDriveSlaveAId = 5;
    public static final int kLeftDriveSlaveBId = 6;
    public static final int kRightDriveMasterId = 1;
    public static final int kRightDriveSlaveAId = 2;
    public static final int kRightDriveSlaveBId = 3;

    // Solenoids
    public static final int kShifterSolenoidId = 12; // PCM 0, Solenoid 4

    // Control Board
    public static final boolean kUseGamepadForDriving = false;
    public static final boolean kUseGamepadForButtons = true;

    public static final int kDriveGamepadPort = 0;
    public static final int kButtonGamepadPort = 2;
    public static final int kMainThrottleJoystickPort = 0;
    public static final int kMainTurnJoystickPort = 1;
    public static final double kJoystickThreshold = 0.5;
    public static final double kJoystickJogThreshold = 0.4;

    //Breaker model for trip time output in seconds y = a*(current_percent_over_rating)^b + c
    public static final double kPDPBreakerModelA = 282.2962;
    public static final double kPDPBreakerModelB = -6.6305;
    public static final double kPDPBreakerModelC = 0.5;
    public static final double kPDPDefaultSafetyFactor = 4.0;







    //Thread prioritization - 5 is default
    public static final int kRobotThreadPriority = 9;
    public static final int kLooperThreadPriority = Thread.MAX_PRIORITY;
    public static final int kCriticalSystemsMonitorThreadPriority = 8;
    public static final int kConnectionMonitorThreadPriority = 7;
    public static final int kLEDThreadPriority = Thread.MIN_PRIORITY;
    public static final int kConsoleReporterThreadPriority = Thread.NORM_PRIORITY;
    public static final int kLogDataReporterThreadPriority = 6;


    public static final String DASHBOARD_IP = "10.1.95.14";
    public static final int DASHBOARD_REPORTER_PORT = 5801;
    public static final int AUTO_SELECTOR_PORT = 5803;
    public static final int LOG_OSC_REPORTER_PORT = 5805;
    public static final int DASHJOY_RECEIVER_PORT = 5806;
    public static final int MOBILE_DIAGNOSTICS_PORT = 5807;


    public static final boolean TUNING_PIDS = true;
    public static final boolean DEBUG = false;
    public static final boolean REPORTING_ENABLED = true;
    public static final boolean REPORT_TO_DRIVERSTATION_INSTEAD_OF_CONSOLE = false;
    public static final RGBColor kDefaultColor = new RGBColor(210, 0, 120);  //Default purple color
    public static final RGBColor kCommLossColor = new RGBColor(255, 0, 0);
    public static final RGBColor kRequestCubeColor = new RGBColor(0, 255, 0);
    public static final RGBColor kGotCubeColor = kDefaultColor;
    public static final RGBColor kElevatorHomeColor = new RGBColor(0, 0, 255);


    public static final boolean ENABLE_DRIVE_DIAG = true;
    public static final boolean ENABLE_CUBE_HANDLER_DIAG = true;
    public static final boolean ENABLE_CLIMBER_DIAG = false;

    //TODO: Tune collision detection
    // Collision Detection
    public static final double kCollisionDetectionJerkThreshold = 950;
    public static final double kTippingThresholdDeg = 11;

    public static final int kActionTimeoutS = 2;

    public static final int kCANifierLEDId = 30;



    /**
     * Make an {@link Solenoid} instance for the single-number ID of the solenoid
     * <p>
     * Solenoids were wired in an inane method and also not labeled zero indexed.
     * <p>
     * Solenoids 1-4 are on PCM 1, Solenoids 7-4.
     * Solenoids 5-8 are on PCM 0, Solenoids 0-3.
     * Solenoids 9-12 are on PCM 0, Solenoids 7-4.
     *
     * @param solenoidId One of the kXyzSolenoidId constants
     */
    public static Solenoid makeSolenoidForId(int solenoidId) {
        if (solenoidId <= 4) {
            // These solenoids are on PCM 1, wired 1-4 to 7-4.
            return new Solenoid(1, 8 - solenoidId);
        } else if (solenoidId <= 8) {
            // These solenoids are on PCM 0, wired 5-8 to 0-3.
            return new Solenoid(0, solenoidId - 5);
        } else if (solenoidId <= 12) {
            // These solenoids are on PCM 0, wired 9-12 to 7-4.
            return new Solenoid(0, 16 - solenoidId);
        }
        throw new IllegalArgumentException("Solenoid ID not valid: " + solenoidId);
    }

    /**
     * @return the MAC address of the robot
     */
    public static String getMACAddress() {
        try {
            Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
            StringBuilder ret = new StringBuilder();
            while (nwInterface.hasMoreElements()) {
                NetworkInterface nis = nwInterface.nextElement();
                if (nis != null) {
                    byte[] mac = nis.getHardwareAddress();
                    if (mac != null) {
                        for (int i = 0; i < mac.length; i++) {
                            ret.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? "-" : ""));
                        }
                        return ret.toString();
                    } else {
                        System.out.println("Address doesn't exist or is not accessible");
                    }
                } else {
                    System.out.println("Network Interface for the specified address is not found.");
                }
            }
        } catch (SocketException e) {
            e.printStackTrace();
        } catch (NullPointerException e) {
            e.printStackTrace();
        }
        return "";
    }
}
