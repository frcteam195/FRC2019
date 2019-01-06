package com.team195.lib.util.drivers.dashjoy;

public class CKDashJoystick {

	private final int mPort;

	private boolean[] prevButtonVal;

	public CKDashJoystick(int port) {
		mPort = port;
		prevButtonVal = new boolean[DashJoyController.BUTTON_ARR_SIZE];

		for (int i = 0; i < prevButtonVal.length; i++) {
			prevButtonVal[i] = false;
		}
	}

	public int getPOV() {
		return getPOV(0);
	}

	public int getPOV(int pov) {
		return DashJoyController.getInstance().getPOV(mPort);
	}

	public double getRawAxis(int axis) {
		return DashJoyController.getInstance().getRawAxis(mPort, axis);
	}


	public boolean getRawButton(int button) {
		return DashJoyController.getInstance().getRawButton(mPort, button-1);
	}

	public boolean getRisingEdgeButton(int button) {
		try {
			boolean currentButton = getRawButton(button);
			boolean retVal = (currentButton != prevButtonVal[button-1]) && currentButton;
			setPrevButtonVal(button-1, currentButton);
			return retVal;
		} catch(Exception ex) {
			return false;
		}
	}

	public boolean getFallingEdgeButton(int button) {
		try {
			boolean currentButton = getRawButton(button);
			boolean retVal = (currentButton != prevButtonVal[button-1]) && !currentButton;
			setPrevButtonVal(button-1, currentButton);
			return retVal;
		} catch(Exception ex) {
			return false;
		}
	}

	private synchronized void setPrevButtonVal(int idx, boolean val) {
		if (idx <= prevButtonVal.length) {
			prevButtonVal[idx] = val;
		}
	}
}
