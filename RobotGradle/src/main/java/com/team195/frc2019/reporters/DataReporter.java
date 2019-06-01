package com.team195.frc2019.reporters;

import com.illposed.osc.OSCListener;
import com.illposed.osc.OSCPacket;
import com.illposed.osc.OSCPortIn;
import com.illposed.osc.OSCPortOut;
import com.team195.frc2019.constants.Constants;

import java.io.IOException;

/**
 * New version of the DataReporter. Requires a heartbeat from each subscriber.
 */
public class DataReporter {
	private static final int portNumber = Constants.LOG_OSC_REPORTER_PORT;

	private static ReportRequestorSet requestorSet = new ReportRequestorSet();
	private static boolean firstRun = true;

	@SuppressWarnings("FieldCanBeLocal")
	private static OSCPortIn oscPortIn;

	private static Runnable initializer = () -> {
		try {
			oscPortIn = new OSCPortIn(portNumber);
			OSCListener updateListener = (time, message) -> {
				try {
					requestorSet.add(message.getIPAddress());
				} catch (Exception ignored) {

				}
			};

			oscPortIn.addListener("/RegisterRequestor", updateListener);
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

	public static synchronized void reportOSCData(OSCPacket oscPacket) {
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
