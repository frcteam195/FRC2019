package com.team195.frc2019;

import com.team195.frc2019.reporters.ConsoleReporter;
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


    //17.25:1
    public static final double kElevatorPositionKp = 1.9;
    public static final double kElevatorPositionKi = 0.0;
    public static final double kElevatorPositionKd = 8.0;
    public static final double kElevatorPositionKf = 0.1380124477;
    public static final int kElevatorPositionCruiseVel = 850;
    public static final int kElevatorPositionMMAccel = 600;
    //Units in rotations
    public static final double kElevatorPositionForwardSoftLimit = 6.3;
    public static final double kElevatorPositionReverseSoftLimit = 0;

    //50:1
    public static final double kTurretPositionKp = 4.3;
    public static final double kTurretPositionKi = 0.0;
    public static final double kTurretPositionKd = 8.0;
    public static final double kTurretPositionKf = 0.400360;
    public static final int kTurretPositionCruiseVel = 350;
    public static final int kTurretPositionMMAccel = 600;

    //TODO: Tune
    //100:1
    public static final double kHatchArmPositionKp = 2.4;
    public static final double kHatchArmPositionKi = 0.0;
    public static final double kHatchArmPositionKd = 6.0;
    public static final double kHatchArmPositionKf = 0.8000721603;
    public static final int kHatchArmPositionCruiseVel = 100;
    public static final int kHatchArmPositionMMAccel = 60;
    //Units in rotations
//    public static final double kHatchArmForwardSoftLimit = 0.6174;
    public static final double kHatchArmForwardSoftLimit = 0.6;

    public static final double kHatchArmReverseSoftLimit = 0;


    //200:1
    public static final double kBallIntakeArmUpPositionKp = 7.3;
    public static final double kBallIntakeArmUpPositionKi = 0.0;
    public static final double kBallIntakeArmUpPositionKd = 100.0;
    public static final double kBallIntakeArmUpPositionKf = 1.6;
    public static final int kBallIntakeArmUpPositionCruiseVel = 90;
    public static final int kBallIntakeArmUpPositionMMAccel = 55;

    //100:1
    public static final double kBallIntakeArmDownPositionKp = 3.7;
    public static final double kBallIntakeArmDownPositionKi = 0.0;
    public static final double kBallIntakeArmDownPositionKd = 12.0;
    public static final double kBallIntakeArmDownPositionKf = 0.8000721603;
    public static final int kBallIntakeArmDownPositionCruiseVel = 160;
    public static final int kBallIntakeArmDownPositionMMAccel = 320;
    //Units in rotations
    public static final double kBallIntakeArmForwardSoftLimit = 0.174;

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
    public static final int kRightDriveMasterId = 1;
	public static final int kRightDriveSlaveAId = 2;
	public static final int kRightDriveSlaveBId = 3;
    public static final int kLeftDriveMasterId = 4;
    public static final int kLeftDriveSlaveAId = 5;
    public static final int kLeftDriveSlaveBId = 6;

    // Elevator
	public static final int kElevatorMasterLeftId = 8;
	public static final int kElevatorSlaveALeftId = 9;
	public static final int kElevatorSlaveBRightId = 7;

	//Ball Intake Arm
	public static final int kBallIntakeRotationMotorId = 10;
	public static final int kBallIntakeRollerMotorId = 11;

	//Hatch Intake Arm
	public static final int kHatchIntakeRotationMotorId = 12;
	public static final int kHatchIntakeRollerMotorId = 13;

	//Turret
	public static final int kTurretMotorId = 14;
	public static final int kBallShooterMotorId = 15;


	// Solenoids
    public static final int kPTOShifterSolenoidId = 0;
    public static final int kBallIntakeBarSolenoidId = 2;
//    public static final int kHatchBeakSolenoidId = 3;
//    public static final int kHatchPushSolenoidId = 5;
//    public static final int kBallPushSolenoidId = 7;
    public static final int kHatchBeakSolenoidId = 9;
    public static final int kHatchBeakFeedSolenoidId = 10;
    public static final int kHatchPushSolenoidId = 8;
    public static final int kBallPushSolenoidId = 11;


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

    public static final double kTurretSmallGearTeeth = 36;
    public static final double kTurretLargeGearTeeth = 252;

    //Breaker model for trip time output in seconds y = a*(current_percent_over_rating)^b + c
    public static final double kPDPBreakerModelA = 282.2962;
    public static final double kPDPBreakerModelB = -6.6305;
    public static final double kPDPBreakerModelC = 0.5;
    public static final double kPDPDefaultSafetyFactor = 4.0;


    public static final double kElevatorLowSensitivityThreshold = 2.5;
    public static final double kLowSensitivityFactor = 0.5;







    //Thread prioritization - 5 is default
    public static final int kRobotThreadPriority = 9;
    public static final int kLooperThreadPriority = Thread.MAX_PRIORITY;
    public static final int kCriticalSystemsMonitorThreadPriority = 8;
    public static final int kConnectionMonitorThreadPriority = 7;
    public static final int kLEDThreadPriority = Thread.MIN_PRIORITY;
    public static final int kConsoleReporterThreadPriority = Thread.NORM_PRIORITY;
    public static final int kLogDataReporterThreadPriority = 4;


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

    public static final int kActionTimeoutS = 10;

    public static final int kCANifierLEDId = 30;

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
                        ConsoleReporter.report("Address doesn't exist or is not accessible");
                    }
                } else {
                    ConsoleReporter.report("Network Interface for the specified address is not found.");
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
