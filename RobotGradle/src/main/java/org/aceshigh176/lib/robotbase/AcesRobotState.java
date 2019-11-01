package org.aceshigh176.lib.robotbase;

import edu.wpi.first.wpilibj.DriverStation;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import java.util.function.Supplier;

public class AcesRobotState {

    private static final Logger log = LogManager.getLogger(AcesRobotState.class);

    interface IAcesRobotState {
        boolean isDisabled();
        boolean isEnabled();
        boolean isOperatorControl();
        boolean isAutonomous();
        boolean isTest();
    }

    private static final IAcesRobotState mDriverStationImpl = new IAcesRobotState() {
        public boolean isDisabled() {
            return DriverStation.getInstance().isDisabled();
        }
        public boolean isEnabled() {
            return DriverStation.getInstance().isEnabled();
        }
        public boolean isOperatorControl() {
            return DriverStation.getInstance().isOperatorControl();
        }
        public boolean isAutonomous() {
            return DriverStation.getInstance().isAutonomous();
        }
        public boolean isTest() {
            return DriverStation.getInstance().isTest();
        }
    };

//    private static final IAcesRobotState mSupplierImpl =

    public static void setImpl(Supplier<RobotOperationalMode> modeSupplier) {
        mImpl = new IAcesRobotState() {
            public boolean isDisabled() {
                return modeSupplier.get() == RobotOperationalMode.kDisabled;
            }
            public boolean isEnabled() {
                return !isDisabled();
//                return modeSupplier.get() == RobotOperationalMode.kAutonomous ||
//                        modeSupplier.get() == RobotOperationalMode.kTeleop;
            }
            public boolean isOperatorControl() {
                return modeSupplier.get() == RobotOperationalMode.kTeleop;
            }
            public boolean isAutonomous() {
                return modeSupplier.get() == RobotOperationalMode.kAutonomous;
            }
            public boolean isTest() {
                return modeSupplier.get() == RobotOperationalMode.kTest;
            }
        };
    }

    private static IAcesRobotState mImpl = mDriverStationImpl;

    public static boolean isDisabled() {
        return mImpl.isDisabled();
    }

    public static boolean isEnabled() {
        return mImpl.isEnabled();
    }

    public static boolean isOperatorControl() {
        return mImpl.isOperatorControl();
    }

    public static boolean isAutonomous() {
        return mImpl.isAutonomous();
    }

    public static boolean isTest() {
        return mImpl.isTest();
    }

    public static RobotOperationalMode getOperationalMode() {
        if(mImpl.isOperatorControl()) {
            return RobotOperationalMode.kTeleop;
        } else if(mImpl.isAutonomous()) {
            return RobotOperationalMode.kAutonomous;
        } else if(mImpl.isTest()) {
            return RobotOperationalMode.kTest;
        } else if(mImpl.isDisabled()) {
            return RobotOperationalMode.kDisabled;
        } else {
            log.error("No known operation mode for robot!");
            return null;
        }
    }

    private AcesRobotState() {
    }
}
