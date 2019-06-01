package com.team195.frc2019.reporters;

import com.illposed.osc.OSCPortOut;
import com.team195.frc2019.constants.Constants;
import com.team195.lib.util.TimeoutTimer;

import java.net.InetAddress;

public class ReportRequestor {
	private final InetAddress inetAddress;
	private OSCPortOut oscPortOut;
	private final TimeoutTimer timeoutTimer;

	private static final int portNumber = Constants.LOG_OSC_REPORTER_PORT;

	public ReportRequestor(InetAddress inetAddress) {
		this(inetAddress, 4);
	}

	public ReportRequestor(InetAddress inetAddress, double heartbeatTimeout) {
		this.inetAddress = inetAddress;
		try {
			oscPortOut = new OSCPortOut(inetAddress, portNumber);
		} catch (Exception ex) {
			oscPortOut = null;
		}
		timeoutTimer = new TimeoutTimer(heartbeatTimeout);
	}

	public InetAddress getInetAddress() {
		return inetAddress;
	}

	public OSCPortOut getOscPortOut() {
		return oscPortOut;
	}

	public synchronized void setOscPortOut(OSCPortOut oscPortOut) {
		this.oscPortOut = oscPortOut;
	}

	public synchronized boolean isExpired() {
		return timeoutTimer.isTimedOut();
	}

	public synchronized void pumpHeartbeat() {
		timeoutTimer.reset();
	}

	@Override
	public boolean equals(Object o) {
		if(o instanceof ReportRequestor){
			return inetAddress.equals(((ReportRequestor) o).inetAddress);
		} else if (o instanceof InetAddress) {
			return inetAddress.equals(o);
		}
		return false;
	}

	@Override
	public int hashCode() {
		return inetAddress.hashCode();
	}
}
