package com.team195.frc2019.coprocessor;

import org.aceshigh176.lib.robotbase.RobotOperationalMode;

public class RioToCoDataStreamerData {

    private static final RioToCoDataStreamerData mInstance = new RioToCoDataStreamerData();

    public static RioToCoDataStreamerData getInstance() {
        return mInstance;
    }
    private RioToCoDataStreamerData() {}

    public double timestamp = 0;
    public RobotOperationalMode robotOperationalMode = RobotOperationalMode.kDisabled;

}
