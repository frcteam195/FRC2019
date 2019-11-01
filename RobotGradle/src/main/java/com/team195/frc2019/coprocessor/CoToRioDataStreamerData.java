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
					if (valArr.size() == 1) {
						mRotation = (float)valArr.get(0);
					}
				} catch (Exception ignored) {

				}
			};

			OSCListener modeListener = (time, message) -> {
				try {
					List<Object> valArr = message.getArguments();
					if (valArr.size() == 1) {
						robotOperationalMode = RobotOperationalMode.valueOf(valArr.get(0));
					}
				} catch (Exception ignored) {

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
				requestorSet.add(InetAddress.getByName("10.1.95.2"));
			} catch (Exception ex) {

			}
		}

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
