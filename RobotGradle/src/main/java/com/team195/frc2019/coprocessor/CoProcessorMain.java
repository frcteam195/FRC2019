package com.team195.frc2019.coprocessor;

import org.aceshigh176.lib.robotbase.AcesPiIterativeRobotBase;
import org.aceshigh176.lib.robotbase.AcesRobotState;

public class CoProcessorMain {
    private CoProcessorMain() {}

    public static void main(String[] args) {
        AcesRobotState.setImpl(() -> RioToCoDataStreamerData.getInstance().robotOperationalMode);
        AcesPiIterativeRobotBase.startRobot(CoProcessor::new);


    }

}
