package com.team195.frc2019.coprocessor;

import org.aceshigh176.lib.externalactions.ArbitraryCodeExecutorServer;
import org.aceshigh176.lib.robotbase.AcesPiTimedRobot;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class CoProcessor extends AcesPiTimedRobot {

    private static final Logger log = LogManager.getLogger(CoProcessor.class);

    private final RioToCoDataStreamerData mRioToCoDataStreamerData = RioToCoDataStreamerData.getInstance();
    private final ArbitraryCodeExecutorServer mArbitraryCodeExecutionClient = new ArbitraryCodeExecutorServer();

    @Override
    public void robotInit() {
        log.info("robotInit called");
    }

    @Override
    public void robotPeriodic() {
    }

    @Override
    public void autonomousInit() {
        log.info("autoInit called");

    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        log.info("teleopInit called");

    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void disabledInit() {
        log.info("disabledInit called");

    }

    @Override
    public void disabledPeriodic() {

    }
}
