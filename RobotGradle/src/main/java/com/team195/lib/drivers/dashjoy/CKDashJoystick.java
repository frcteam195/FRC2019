package com.team195.lib.drivers.dashjoy;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

public class CKDashJoystick {

	private final int mPort;

	private boolean[] prevButtonVal;

	private Joystick backupJoystick;

	private DashJoyController dashJoyController = DashJoyController.getInstance();

	//Expire after 20ms, value in microseconds
	private static final long EXPIRATION_TIME = 20000;

	public CKDashJoystick(int port) {
		mPort = port;
		prevButtonVal = new boolean[DashJoyController.BUTTON_ARR_SIZE];

		for (int i = 0; i < prevButtonVal.length; i++) {
			prevButtonVal[i] = false;
		}

		backupJoystick = new Joystick(mPort);
	}

	public int getPOV() {
		return getPOV(0);
	}

	public int getPOV(int pov) {
		if (isTimestampValid())
			return dashJoyController.getPOV(mPort);
		else
			return backupJoystick.getPOV();
	}

	public double getRawAxis(int axis) {
		if (isTimestampValid())
			return dashJoyController.getRawAxis(mPort, axis);
		else
			return backupJoystick.getRawAxis(axis);
	}


	public boolean getRawButton(int button) {
		if (isTimestampValid())
			return dashJoyController.getRawButton(mPort, button-1);
		else
			return backupJoystick.getRawButton(button);
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

	private boolean isTimestampValid() {
		return (HALUtil.getFPGATime() - dashJoyController.getLastUpdateTimestamp()) < EXPIRATION_TIME;
	}
}
