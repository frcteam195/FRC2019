package com.team195.lib.drivers.dashjoy;

import java.util.ArrayList;

public class DashJoyController {
	private ArrayList<DashJoyData> joystickDataArrayList = new ArrayList<>();

	private static DashJoyController instance = null;
	private static final int MAX_CHANNELS = 6;
	public static final int BUTTON_ARR_SIZE = 64;

	public static DashJoyController getInstance() {
		if (instance == null)
			instance = new DashJoyController();

		return instance;
	}

	private DashJoyController() {
		for (int i = 0; i < MAX_CHANNELS; i++) {
			joystickDataArrayList.add(new DashJoyData());
		}
	}

	public void setRawAxis(int channel, int axis, double val) {
		if (channel < MAX_CHANNELS && channel >= 0)
			joystickDataArrayList.get(channel).setAxisValue(axis, val);
	}

	public void setRawButton(int channel, int button, boolean val) {
		if (channel < MAX_CHANNELS && channel >= 0)
			joystickDataArrayList.get(channel).setButtonValue(button, val);
	}

	public void setPOV(int channel, int val) {
		if (channel < MAX_CHANNELS && channel >= 0)
			joystickDataArrayList.get(channel).setPOVValue(val);
	}

	public double getRawAxis(int channel, int axis) {
		if (channel < MAX_CHANNELS && channel >= 0)
			return joystickDataArrayList.get(channel).getAxisValue(axis);

		return 0;
	}

	public boolean getRawButton(int channel, int button) {
		if (channel < MAX_CHANNELS && channel >= 0)
			return joystickDataArrayList.get(channel).getButtonValue(button);

		return false;
	}

	public int getPOV(int channel) {
		if (channel < MAX_CHANNELS && channel >= 0)
			return joystickDataArrayList.get(channel).getPOVValue();

		return 0;
	}
}
