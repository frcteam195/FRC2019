package com.team195.frc2019.coprocessor;

import com.illposed.osc.*;
import com.revrobotics.CANSparkMaxFrames;
import com.team195.frc2019.constants.Constants;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.reporters.ReportRequestorSet;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import org.aceshigh176.lib.robotbase.RobotOperationalMode;

import java.io.IOException;
import java.net.InetAddress;
import java.util.List;

public class CoToRioDataStreamerData {

	private static final CoToRioDataStreamerData mInstance = new CoToRioDataStreamerData();

	public static CoToRioDataStreamerData getInstance() {
		return mInstance;
	}
	private CoToRioDataStreamerData() {}

	public double timestamp = 0;
	public RobotOperationalMode robotOperationalMode = RobotOperationalMode.kDisabled;

	private final int portNumber = Constants.LOG_RIOOSC_REPORTER_PORT;

	private ReportRequestorSet requestorSet = new ReportRequestorSet();
	private boolean firstRun = true;

	private double mRotation = 0;

	@SuppressWarnings("FieldCanBeLocal")
	private OSCPortIn oscPortIn;

	public double getCurrentHeading() {
		return mRotation;
	}

	private Runnable initializer = () -> {
		try {
			oscPortIn = new OSCPortIn(portNumber);

			OSCListener gyroListener = (time, message) -> {
				try {
					List<Object> valArr = message.getArguments();
					System.out.println("gyroLister fired " + valArr.size());
					if (valArr.size() == 1) {
						mRotation = (double)(valArr.get(0));
					}
				} catch (Exception ignored) {
					ignored.printStackTrace();
				}
			};

			OSCListener modeListener = (time, message) -> {
				try {
					List<Object> valArr = message.getArguments();
					System.out.println("modeLister fired " + valArr.size());
					if (valArr.size() == 1) {
						robotOperationalMode = RobotOperationalMode.valueOf(valArr.get(0));
					}
				} catch (Exception ignored) {
					ignored.printStackTrace();
				}
			};

			oscPortIn.addListener("/GyroData", gyroListener);
			oscPortIn.addListener("/RobotMode", modeListener);
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
			try {
				requestorSet.add(InetAddress.getByName("10.1.95.2"), portNumber);
			} catch (Exception ex) {
				ex.printStackTrace();
			}
		}

		requestorSet.forEach((k) -> {
			if (k.getInetAddress() != null) {
				if (k.getOscPortOut() == null) {
					try {
						System.out.println("SetOSCPort");
						k.setOscPortOut(new OSCPortOut(k.getInetAddress(), portNumber));
					} catch (Exception ignored) {
						System.out.println("Error");
						return;
					}
				}
				try {
					//System.out.println(k.getInetAddress().getHostAddress());
					if (oscPacket instanceof OSCBoundListMessage) {
						//System.out.println("Send message " + ((OSCBoundListMessage)oscPacket).getAddress());
					} else {
						//System.out.println("Send message " + ((OSCMessage)oscPacket).getAddress());
					}
					k.getOscPortOut().send(oscPacket);
				} catch (IOException ignored) {

				}
			}
		});
	}
}
