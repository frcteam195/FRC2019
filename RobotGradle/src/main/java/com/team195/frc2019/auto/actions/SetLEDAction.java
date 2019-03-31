package com.team195.frc2019.auto.actions;

import com.team195.frc2019.controllers.LEDController;
import com.team195.lib.util.RGBColor;

public class SetLEDAction implements Action {
	private static final LEDController mLEDController = LEDController.getInstance();

	private final RGBColor mLEDColor;
	private final LEDController.LEDState mLEDState;

	public SetLEDAction(RGBColor ledColor) {
		this(ledColor, LEDController.LEDState.BLINK);
	}

	public SetLEDAction(RGBColor ledColor, LEDController.LEDState ledState) {
		mLEDColor = ledColor;
		mLEDState = ledState;
	}

	@Override
	public boolean isFinished() {
		return true;
	}

	@Override
	public void update() {

	}

	@Override
	public void done() {

	}

	@Override
	public void start() {
		mLEDController.setLEDColor(mLEDColor);
		mLEDController.setRequestedState(mLEDState);
	}
}
