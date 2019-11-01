package org.aceshigh176.lib.util;

public class UnitUtil {

    private static final double kNanosecondsToSeconds = Math.pow(10, -9);
    private static final double kNanosecondsToMilliseconds = Math.pow(10, -6);
    private static final double kMicrosecondsToSeconds = Math.pow(10, -6);
    private static final double kMillisecondsToSeconds = Math.pow(10, -3);

    /**
     * Calculate the rotations needed to travel <code>distance</code> given <code>wheelDiameter</code>
     *
     * @param distance the distance travelled
     * @param wheelDiameter in same units as distance
     */
    public static double distanceToRotations(double distance, double wheelDiameter) {
        // C = pi * d
        // rot = distance / C
        return distance / (Math.PI * wheelDiameter);
    }

    /**
     * Calculate the distance traveled over <code>rotations</code> rotations given <code>wheelDiameter</code>
     *
     * @param rotations
     * @param wheelDiameter in units of return type
     */
    public static double rotationsToDistance(double rotations, double wheelDiameter) {
        // C = pi * d
        // rot * C =  inches
        return rotations * (Math.PI * wheelDiameter);
    }

    public static double rotationsToRadians(double rotations) {
        return rotations * 2 * Math.PI;
    }

    public static double radiansToRotations(double radians) {
        return radians / (2 * Math.PI);
    }

    /**
     * Note: This method currently doesn't work
     *
     * @param desiredVelocity
     * @param driveWheelDiameter
     * @param driveKv
     * @return
     */
    public static double solveForOpenLoopVelocity(double desiredVelocity, double driveWheelDiameter, double driveKv) {
        double desiredVelocityRadsPerSec = UnitUtil.rotationsToRadians(UnitUtil.distanceToRotations(desiredVelocity, driveWheelDiameter));
        System.out.println(desiredVelocityRadsPerSec);
        // How many volts do I need to achieve this?
        double requiredVoltsForVelocity = desiredVelocityRadsPerSec / driveKv /* V per rad/s */;
        double requiredOutputForVelocity = requiredVoltsForVelocity / 12.0 /* volts */;
//        return UnitUtil.rotationsToDistance(UnitUtil.radiansToRotations(Constants.kDriveKv /* V per rad/s */), Constants.kDriveWheelDiameterInches) / 12.0 /* vbus volts */;
        return requiredVoltsForVelocity;
    }

    public static double hzToPeriod(int hz) {
        return hzToPeriod((double) hz);
    }

    public static double hzToPeriod(double hz) {
        return 1 / hz;
    }

    public static double periodToHz(double period) {
        return 1 / period;
    }

    public static double nanosecondsToSeconds(long nanoseconds) {
        return nanoseconds * kNanosecondsToSeconds;
    }

    public static double secondsToNanoseconds(double seconds) {
        return seconds / kNanosecondsToSeconds;
    }

    public static double microsecondsToSeconds(long microseconds) {
        return microseconds * kMicrosecondsToSeconds;
    }

    public static double secondsToMicroseconds(double seconds) {
        return seconds / kMicrosecondsToSeconds;
    }

    public static double millisecondsToSeconds(long milliseconds) {
        return milliseconds * kMillisecondsToSeconds;
    }

    public static double secondsToMilliseconds(double seconds) {
        return seconds / kMillisecondsToSeconds;
    }

    public static long nanosecondsToMilliseconds(int ns) {
        return (long) (ns * kNanosecondsToMilliseconds);
    }

    //https://stackoverflow.com/questions/3758606/how-to-convert-byte-size-into-human-readable-format-in-java
    public static String humanReadableByteCount(long bytes, boolean si) {
        int unit = si ? 1000 : 1024;
        if (bytes < unit) return bytes + " B";
        int exp = (int) (Math.log(bytes) / Math.log(unit));
        String pre = (si ? "kMGTPE" : "KMGTPE").charAt(exp-1) + (si ? "" : "i");
        return String.format("%.1f %sB", bytes / Math.pow(unit, exp), pre);
    }

}
