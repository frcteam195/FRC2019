package com.team195.lib.util.drivers.dashjoy;

import com.illposed.osc.OSCListener;
import com.illposed.osc.OSCMessage;
import com.illposed.osc.OSCPortIn;
import com.team254.lib.util.MovingAverage;

import java.util.List;

public class DashJoyReceiver {
	private static final int RECEIVE_PORT = 5806;

	private OSCPortIn oscPortIn;
	public MovingAverage movingAverage = new MovingAverage(50);

	private static DashJoyReceiver instance = null;

	public static DashJoyReceiver getInstance() {
		if (instance == null)
			instance = new DashJoyReceiver();

		return instance;
	}

	private DashJoyReceiver() {
		try {
			oscPortIn = new OSCPortIn(RECEIVE_PORT);

			OSCListener joystickListener = new OSCListener() {
				// Joystick Packing iiiiiihi
				// Axes 1-6 as int
				// Buttons as 64bit long
				// POV as int
				public void acceptMessage(java.util.Date time, OSCMessage message) {
					try {
						int joystickNum;
						try {
							joystickNum = Integer.parseInt(message.getAddress().split("/")[2]);
						}
						catch (Exception ex) {
							joystickNum = -1;
						}
						if (joystickNum == -1)
							return;

						List<Object> valArr = message.getArguments();

						for (int i = 0; i < 6; i++) {
							double axisVal = ((int)valArr.get(i))/32767.0;
							axisVal = axisVal > 1 ? 1 : axisVal;
							axisVal = axisVal < -1 ? -1 : axisVal;
							DashJoyController.getInstance().setRawAxis(joystickNum, i, axisVal);
						}

						boolean[] buttons = getButtonArrFromLong((long)valArr.get(6));
						for (int i = 0; i < buttons.length; i++) {
							DashJoyController.getInstance().setRawButton(joystickNum, i, buttons[i]);
						}

						int pov = (int)valArr.get(7);
						pov = pov == -1 ? -1 : pov / 100;
						DashJoyController.getInstance().setPOV(joystickNum, pov);

						long remoteTimestamp = (long)valArr.get(8);
						long currentTimestamp = System.currentTimeMillis();
						movingAverage.addNumber(currentTimestamp-remoteTimestamp);
					} catch (Exception ex) {
						ex.printStackTrace();
					}
				}
			};
			oscPortIn.addListener("/Joysticks/*", joystickListener);



			oscPortIn.startListening();
		} catch (Exception ex) {

		}
	}

	private boolean[] getButtonArrFromLong(long buttons) {
		boolean[] arr = new boolean[64];
		for (int i = 0; i < arr.length; i++) {
			arr[i] = ((buttons >> i) & 0x01) == 1;
		}
		return arr;
	}


}
