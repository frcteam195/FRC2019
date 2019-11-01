package org.aceshigh176.lib.util;

import com.team195.lib.util.ARMNotifierJNI;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpiutil.RuntimeDetector;

public class AcesRobotStatus {

    private static StatusSource mStatusSource;
    private static SleepImpl mSleepImpl;

    private static final StatusSource mStatusSourceRio = new AcesRobotStatus.StatusSource() {
        @Override
        public double getFPGATimestamp() {
            return RobotController.getFPGATime() / 1000000.0;
        }
    };

    private static final double programStartTime;
    private static final StatusSource mStatusSourceSW = new AcesRobotStatus.StatusSource() {
        @Override
        public double getFPGATimestamp() {
            return (UnitUtil.nanosecondsToSeconds(System.nanoTime()) - programStartTime);
        }
    };

    private static final SleepImpl mSleepImplThread = new AcesRobotStatus.SleepImpl() {
        @Override
        public void sleepFor(int ns) {
            try {
                Thread.sleep(UnitUtil.nanosecondsToMilliseconds(ns));
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    };

    private static final SleepImpl mSleepImplArmNotifierJNI = new AcesRobotStatus.SleepImpl() {
        @Override
        public void sleepFor(int ns) {
            ARMNotifierJNI.sleepForNano(ns);
        }
    };

    static {
        String platformPath = RuntimeDetector.getPlatformPath();
        System.out.println("Detected current platform path as '" + platformPath + "'");
        programStartTime = UnitUtil.nanosecondsToSeconds(System.nanoTime());
        if(platformPath.contains("athena")) {
            // Running on the RIO
            setStatusSource(mStatusSourceRio);
            setSleepImpl(mSleepImplThread);
        } else if(platformPath.contains("/linux/raspbian/")) {
            setStatusSource(mStatusSourceSW);
            setSleepImpl(mSleepImplArmNotifierJNI);
        } else {
            setStatusSource(mStatusSourceSW);
            setSleepImpl(mSleepImplThread);
        }
    }


//    private static DataStore_RobotStatus mRobotStatusDataStore = new DataStore_RobotStatus();

    public static void setStatusSource(StatusSource source) {
        mStatusSource = source;
    }

    public static void setSleepImpl(SleepImpl impl) {
        mSleepImpl = impl;
    }

    /**
     * Return the system clock time in seconds. Return the time from the FPGA hardware clock in
     * seconds since the FPGA started.
     *
     * @return Robot running time in seconds.
     */
    @SuppressWarnings("AbbreviationAsWordInName")
    public static double getFPGATimestamp() {
        return mStatusSource.getFPGATimestamp();
    }

    public static void sleepFor(int ns) {
        mSleepImpl.sleepFor(ns);
    }

//    public static void updateTelemetry(double timestamp) {
//        mRobotStatusDataStore.fetchDuration.startTimer();
//        mRobotStatusDataStore.timestamp.put((float) timestamp);
//        mRobotStatusDataStore.batteryVoltage.put((float) RobotController.getBatteryVoltage());
//        mRobotStatusDataStore.current3v3.put((float) RobotController.getCurrent3V3());
//        mRobotStatusDataStore.current5v.put((float) RobotController.getCurrent5V());
//        mRobotStatusDataStore.current6v.put((float) RobotController.getCurrent6V());
//        mRobotStatusDataStore.enabled3v3.put(RobotController.getEnabled3V3());
//        mRobotStatusDataStore.enabled5v.put(RobotController.getEnabled5V());
//        mRobotStatusDataStore.enabled6v.put(RobotController.getEnabled6V());
//        mRobotStatusDataStore.faultCount3v3.put(RobotController.getFaultCount3V3());
//        mRobotStatusDataStore.faultCount5v.put(RobotController.getFaultCount5V());
//        mRobotStatusDataStore.faultCount6v.put(RobotController.getFaultCount6V());
//        mRobotStatusDataStore.inputCurrent.put((float) RobotController.getInputCurrent());
//        mRobotStatusDataStore.inputVoltage.put((float) RobotController.getInputVoltage());
//        mRobotStatusDataStore.userButton.put(RobotController.getUserButton());
//        mRobotStatusDataStore.voltage3v3.put((float) RobotController.getVoltage3V3());
//        mRobotStatusDataStore.voltage5v.put((float) RobotController.getVoltage5V());
//        mRobotStatusDataStore.voltage6v.put((float) RobotController.getVoltage6V());
//        mRobotStatusDataStore.isBrownedOut.put(RobotController.isBrownedOut());
//        mRobotStatusDataStore.isSysActive.put(RobotController.isSysActive());
//        mRobotStatusDataStore.fetchDuration.stopTimer();
//    }
//
//    public static DataStore_RobotStatus getRobotStatusDataStore() {
//       return mRobotStatusDataStore;
//    }

    public interface StatusSource {
        double getFPGATimestamp();
    }

    public interface SleepImpl {
        void sleepFor(int ns);
    }

//    public static class DataStore_RobotStatus extends GroupDataStore {
//        public DataStore_Float timestamp = new DataStore_Float();
//        public DataStore_Duration fetchDuration = new DataStore_Duration();
//
//        public DataStore_Float batteryVoltage = new DataStore_Float();
//        public DataStore_Float inputVoltage = new DataStore_Float();
//        public DataStore_Float inputCurrent = new DataStore_Float();
//        public DataStore_Boolean userButton = new DataStore_Boolean();
//        public DataStore_Boolean isBrownedOut = new DataStore_Boolean();
//        public DataStore_Boolean isSysActive = new DataStore_Boolean();
//
//        public DataStore_Boolean enabled3v3 = new DataStore_Boolean();
//        public DataStore_Boolean enabled5v = new DataStore_Boolean();
//        public DataStore_Boolean enabled6v = new DataStore_Boolean();
//        public DataStore_Float voltage3v3 = new DataStore_Float();
//        public DataStore_Float voltage5v = new DataStore_Float();
//        public DataStore_Float voltage6v = new DataStore_Float();
//        public DataStore_Float current3v3 = new DataStore_Float();
//        public DataStore_Float current5v = new DataStore_Float();
//        public DataStore_Float current6v = new DataStore_Float();
//        public DataStore_Integer faultCount3v3 = new DataStore_Integer();
//        public DataStore_Integer faultCount5v = new DataStore_Integer();
//        public DataStore_Integer faultCount6v = new DataStore_Integer();
//    }

}




