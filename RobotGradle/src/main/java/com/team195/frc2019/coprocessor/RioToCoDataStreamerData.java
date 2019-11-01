package com.team195.frc2019.coprocessor;

import com.illposed.osc.OSCListener;
import com.illposed.osc.OSCPacket;
import com.illposed.osc.OSCPortIn;
import com.illposed.osc.OSCPortOut;
import com.team195.frc2019.constants.Constants;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.reporters.ReportRequestorSet;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import org.aceshigh176.lib.robotbase.RobotOperationalMode;

import java.io.IOException;
import java.util.List;

public class RioToCoDataStreamerData {

    private static final RioToCoDataStreamerData mInstance = new RioToCoDataStreamerData();

    public static RioToCoDataStreamerData getInstance() {
        return mInstance;
    }
    private RioToCoDataStreamerData() {}

    public double timestamp = 0;

	private final int portNumber = Constants.LOG_RIOOSC_REPORTER_PORT;

	private ReportRequestorSet requestorSet = new ReportRequestorSet();
	private boolean firstRun = true;

	private Pose2d mPose = new Pose2d();

	@SuppressWarnings("FieldCanBeLocal")
	private OSCPortIn oscPortIn;

	public Pose2d getCurrentPose() {
		return mPose;
	}

	private Runnable initializer = () -> {
		try {
			oscPortIn = new OSCPortIn(portNumber);
			OSCListener updateListener = (time, message) -> {
				try {
					requestorSet.add(message.getIPAddress());
				} catch (Exception ignored) {

				}
			};

			OSCListener poseListener = (time, message) -> {
				try {
					List<Object> valArr = message.getArguments();
					if (valArr.size() == 3) {
						mPose = new Pose2d((float)valArr.get(0), (float)valArr.get(1), Rotation2d.fromRadians((float)valArr.get(2)));
					}
				} catch (Exception ignored) {

				}
			};

			oscPortIn.addListener("/RegisterRequestor", updateListener);
			oscPortIn.addListener("/SLAMPose", poseListener);
			oscPortIn.startListening();
			firstRun = false;
		} catch (Exception ex) {
			if (oscPortIn != null) {
				try {
					oscPortIn.stopListening();
					oscPortIn.close();
				} catch (Exception ignored) {

				}
			}
			oscPortIn = null;

			firstRun = true;
			ConsoleReporter.report(ex);
		}
	};

	@SuppressWarnings("DuplicatedCode")
	public synchronized void reportOSCData(OSCPacket oscPacket) {
		if (firstRun) {
			initializer.run();
		}

		requestorSet.removeExpiredEntries();
		requestorSet.forEach((k) -> {
			if (k.getInetAddress() != null) {
				if (k.getOscPortOut() == null) {
					try {
						k.setOscPortOut(new OSCPortOut(k.getInetAddress(), portNumber));
					} catch (Exception ignored) {
						return;
					}
				}
				try {
					k.getOscPortOut().send(oscPacket);
				} catch (IOException ignored) {

				}
			}
		});
	}
}
