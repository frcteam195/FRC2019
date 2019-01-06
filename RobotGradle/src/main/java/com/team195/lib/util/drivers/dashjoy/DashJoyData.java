package com.team195.lib.util.drivers.dashjoy;

public class DashJoyData {
	double[] axisVal = new double[6];
	boolean[] buttonVal = new boolean[64];
	int pov = -1;

	public synchronized void setAxisValue(int axis, double val) {
		if (axis < axisVal.length && axis >= 0)
			this.axisVal[axis] = val;
	}

	public synchronized void setButtonValue(int button, boolean val) {
		if (button < buttonVal.length && button >= 0)
			this.buttonVal[button] = val;
	}

	public synchronized void setPOVValue(int val) {
		this.pov = val;
	}

	public synchronized double getAxisValue(int axis) {
		if (axis < axisVal.length && axis >= 0)
			return this.axisVal[axis];

		return 0;
	}

	public synchronized boolean getButtonValue(int button) {
		if (button < buttonVal.length && button >= 0)
			return this.buttonVal[button];

		return false;
	}

	public synchronized int getPOVValue() {
		return pov;
	}
}
