package com.team195.lib.util;

import edu.wpi.first.wpilibj.Timer;

public class ElapsedTimer {
	private double startTime = 0;
	
	public ElapsedTimer() {
		
	}
	
	public synchronized void start() {
		startTime = Timer.getFPGATimestamp();
	}
	
	public double hasElapsed() {
		return Timer.getFPGATimestamp() - startTime;
	}
}
