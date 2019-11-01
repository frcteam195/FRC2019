package com.team195.frc2019.coprocessor;

import com.illposed.osc.OSCBoundListMessage;
import com.illposed.osc.OSCMessage;
import org.aceshigh176.lib.externalactions.ArbitraryCodeExecutorServer;
import org.aceshigh176.lib.robotbase.AcesPiTimedRobot;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class CoProcessor extends AcesPiTimedRobot {

    private static final Logger log = LogManager.getLogger(CoProcessor.class);

	private final CoToRioDataStreamerData mCoToRioDataStreamerData = CoToRioDataStreamerData.getInstance();
    private final ArbitraryCodeExecutorServer mArbitraryCodeExecutionClient = new ArbitraryCodeExecutorServer();

    @Override
    public void robotInit() {
        log.info("robotInit called");
    }

	private static final List<Object> headingList = new ArrayList<>(5);
	private static final OSCBoundListMessage boundOSCMesage = new OSCBoundListMessage("/SLAMPose", headingList);
	private static final OSCMessage reportMsg = new OSCMessage("/RegisterRequestor", Collections.emptyList());

    @Override
    public void robotPeriodic() {
		mCoToRioDataStreamerData.reportOSCData(reportMsg);
		headingList.clear();
		//Add Fused Pose Here
		headingList.add((float)0);	//X
		headingList.add((float)0);	//Y
		headingList.add((float)0);	//Rot
		mCoToRioDataStreamerData.reportOSCData(boundOSCMesage);
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
