package com.team195.frc2019.coprocessor;

import com.illposed.osc.OSCBoundListMessage;
import com.illposed.osc.OSCMessage;
import com.team195.lib.util.FastDoubleToString;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.aceshigh176.lib.externalactions.ArbitraryCodeExecutorServer;
import org.aceshigh176.lib.robotbase.AcesPiTimedRobot;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class CoProcessor extends AcesPiTimedRobot {

//    private static final Logger log = LogManager.getLogger(CoProcessor.class);

	private final CoToRioDataStreamerData mCoToRioDataStreamerData = CoToRioDataStreamerData.getInstance();
    private final ArbitraryCodeExecutorServer mArbitraryCodeExecutionClient = new ArbitraryCodeExecutorServer();

    private NetworkTableEntry mOperationalModeEntry;
	private NetworkTableEntry mTimestampEntry;

    @Override
    public void robotInit() {
        System.out.println("robotInit called");
        NetworkTableInstance.getDefault().startServer();
		NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("Data");
		mTimestampEntry = networkTable.getEntry("Timestamp");
		mOperationalModeEntry = networkTable.getEntry("OperationalMode");
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
		mTimestampEntry.setString(FastDoubleToString.format(CoToRioDataStreamerData.getInstance().timestamp));
		mOperationalModeEntry.setString(CoToRioDataStreamerData.getInstance().robotOperationalMode.toString());
    }

    @Override
    public void autonomousInit() {
        System.out.println("autoInit called");

    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        System.out.println("teleopInit called");

    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void disabledInit() {
        System.out.println("disabledInit called");

    }

    @Override
    public void disabledPeriodic() {

    }
}
