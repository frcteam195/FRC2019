package com.team195.frc2019.coprocessor;

import org.aceshigh176.lib.externalactions.ArbitraryCodeExecutorClient;
import org.aceshigh176.lib.externalactions.ArbitraryCodeExecutorLambda;
import org.aceshigh176.lib.robotbase.RobotOperationalMode;

public class CoProcessorRemoteExec {

    private static CoProcessorRemoteExec mInstance;

    public static CoProcessorRemoteExec getInstance() {
        if(mInstance == null) { // Lazy initialization
            mInstance = new CoProcessorRemoteExec();
        }
        return mInstance;
    }

    private final ArbitraryCodeExecutorClient mCodeExecutionClient;

    private CoProcessorRemoteExec() {
        mCodeExecutionClient = new ArbitraryCodeExecutorClient();
    }

    public static class SetRobotData implements ArbitraryCodeExecutorLambda {

        private final RobotOperationalMode mOperationalMode;

        public SetRobotData(RobotOperationalMode robotOperationalMode) {
            mOperationalMode = robotOperationalMode;
        }

        @Override
        public void accept(Void t) {
            RioToCoDataStreamerData.getInstance().robotOperationalMode = mOperationalMode;
        }
    }

    public void setPiData(RobotOperationalMode operationalMode) {
        mCodeExecutionClient.submitForExecution(new SetRobotData(operationalMode));
    }
}
