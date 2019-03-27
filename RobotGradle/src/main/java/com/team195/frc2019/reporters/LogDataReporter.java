package com.team195.frc2019.reporters;


import com.illposed.osc.OSCMessage;
import com.illposed.osc.OSCPortOut;
import com.team195.frc2019.constants.Constants;
import com.team195.frc2019.SubsystemManager;
import com.team195.lib.util.ThreadRateControl;
import edu.wpi.first.wpilibj.DriverStation;

import java.io.IOException;
import java.net.InetAddress;
import java.util.ArrayList;

public class LogDataReporter {
	private static final int portNumber = Constants.LOG_OSC_REPORTER_PORT;
	private static final int MIN_LOG_REPORTER_LOOP_MS = 100;

	private static LogDataReporter instance = null;

	private static InetAddress IPAddress;
	private static OSCPortOut oscPortOut;
	private SubsystemManager subsystemManager = SubsystemManager.getInstance();
	private boolean runThread = true;

	private LogDataReporter() {
		ThreadRateControl threadRateControl = new ThreadRateControl();
		Thread t = new Thread (() -> {
			Thread.currentThread().setName("LogDataReporter");
			threadRateControl.start();
			while (runThread) {
				reportOSCData(subsystemManager.generateReport());
				threadRateControl.doRateControl(MIN_LOG_REPORTER_LOOP_MS);
			}
		});
		t.setPriority(Constants.kLogDataReporterThreadPriority);
		t.start();
	}

	public static LogDataReporter getInstance() {
		if(instance == null) {
			try {
				instance = new LogDataReporter();
			} catch (Exception ex) {
				ConsoleReporter.report(ex, MessageLevel.DEFCON1);
			}
		}

		return instance;
	}

	private static synchronized void reportOSCData(String logData) {
		if (IPAddress != null) {
			if (oscPortOut == null) {
				try {
					oscPortOut = new OSCPortOut(IPAddress, portNumber);
				} catch (Exception ignored) {
					return;
				}
			}
			try {
				oscPortOut.send(createSendData(logData));
			} catch (IOException ignored) {

			}
		} else {
			try {
				IPAddress = InetAddress.getByName(Constants.DASHBOARD_IP);
			} catch (Exception ignored) {

			}
		}
	}

	private static OSCMessage createSendData(String logData) {
		ArrayList<Object> args = new ArrayList<>();
		args.add("Enabled:" + DriverStation.getInstance().isEnabled() + ";");

		String[] sArr = logData.split(";");

		for (String s: sArr)
			if (!s.isEmpty())
				args.add(s + ";");

			//Consumes all CPU. Come up with better way to measure CPU usage
//		try {
//			String[] procCmd = {
//					"/bin/sh",
//					"-c",
//					"grep 'cpu ' /proc/stat | awk '{usage=($2+$4)*100/($2+$4+$5)} END {print usage \"%\"}'"
//			};
//			Process cpuProc = Runtime.getRuntime().exec(procCmd);
//
//			cpuProc.waitFor(10, TimeUnit.MILLISECONDS);
//			String s;
//			StringBuilder sb = new StringBuilder();
//			BufferedReader stdInput = new BufferedReader(new InputStreamReader(cpuProc.getInputStream()));
//			while ((s = stdInput.readLine()) != null) {
//				sb.append(s);
//			}
//
//			args.add("CPULoad_System:" + sb.toString() + ";");
//
//
//
//			String[] memCmd = {
//					"/bin/sh",
//					"-c",
//					"cat /proc/meminfo | awk '/Mem.*\\s*[0-9](\\d*)/{print $2}'"
//			};
//			Process memProc = Runtime.getRuntime().exec(memCmd);
//
//			memProc.waitFor(10, TimeUnit.MILLISECONDS);
//			s = "";
//			int[] memArray = new int[3];	//Total, Free, Available mem
//			stdInput = new BufferedReader(new InputStreamReader(memProc.getInputStream()));
//			for (int i = 0; i < 3; i++) {
//				if ((s = stdInput.readLine()) != null)
//					memArray[i] = Integer.parseInt(s);
//			}
//
//			args.add("TotalMemory:" + memArray[0] + ";");
//			args.add("FreeMemory:" + memArray[1] + ";");
//			args.add("AvailableMemory:" + memArray[2] + ";");
//		} catch (Exception e) {
//
//		}

		return new OSCMessage("/LogData", args);
	}
}
