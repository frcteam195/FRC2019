package com.team195.lib.drivers.dashjoy;

import com.team195.frc2019.Constants;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

public class CKDashJoystick {

	private final int mPort;

	private boolean[] prevButtonVal;
	private boolean[] prevTriggerVal;

	private Joystick backupJoystick;

	private DashJoyController dashJoyController = DashJoyController.getInstance();

	//Expire after 20ms, value in microseconds
	private static final long EXPIRATION_TIME = 20000;

	public CKDashJoystick(int port) {
		mPort = port;
		prevButtonVal = new boolean[DashJoyController.BUTTON_ARR_SIZE];
		prevTriggerVal = new boolean[DashJoyController.MAX_AXES];

		for (int i = 0; i < prevButtonVal.length; i++) {
			prevButtonVal[i] = false;
		}

		for (int i = 0; i < prevTriggerVal.length; i++) {
			prevTriggerVal[i] = false;
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

	public double getNormalizedAxis(int axis, double deadband) {
		return normalizeJoystickWithDeadband(getRawAxis(axis), deadband);
	}

	public double getSmoothedAxis(int axis, double deadband, double power) {
		double x = getRawAxis(axis);
		double sign = Math.signum(x);
		return sign * Math.min(Math.pow(Math.abs(normalizeJoystickWithDeadband(x, deadband)), power), 1);
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

	private synchronized void setPrevTriggerVal(int idx, boolean val) {
		if (idx <= prevTriggerVal.length) {
			prevTriggerVal[idx] = val;
		}
	}

	private boolean isTimestampValid() {
		return (HALUtil.getFPGATime() - dashJoyController.getLastUpdateTimestamp()) < EXPIRATION_TIME;
	}

	private double normalizeJoystickWithDeadband(double val, double deadband) {
		val = (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;

		if (val != 0)
			val = Math.signum(val) * ((Math.abs(val) - deadband) / (1.0 - deadband));

		return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
	}

	public boolean isAxisInputActive() {
		for (int i = 0; i < backupJoystick.getAxisCount(); i++) {
			if (Math.abs(getRawAxis(i)) > Constants.kJoystickJogThreshold) {
				return true;
			}
		}
		return false;
	}

	public boolean isButtonInputActive() {
		for (int i = 1; i <= backupJoystick.getButtonCount(); i++) {
			if (getRawButton(i)) {
				return true;
			}
		}
		return false;
	}

	public boolean getRisingEdgeTrigger(int axis, double threshold) {
		try {
			boolean currentButton = Math.abs(getRawAxis(axis)) > threshold;
			boolean retVal = (currentButton != prevTriggerVal[axis]) && currentButton;
			setPrevTriggerVal(axis, currentButton);
			return retVal;
		} catch(Exception ex) {
			return false;
		}
	}

	public boolean isPOVInputActive() {
		return getPOV() != -1;
	}

	public boolean isJoystickInputActive() {
		return isAxisInputActive() || isButtonInputActive() || isPOVInputActive();
	}
}
