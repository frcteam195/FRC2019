package com.team195.lib.drivers;

import com.team195.lib.util.RGBColor;
import edu.wpi.first.wpilibj.DigitalOutput;

public class LEDDriverRGB implements LEDDriver{
	private static final int PWM_FREQ = 500;

	private DigitalOutput rLED;
	private DigitalOutput gLED;
	private DigitalOutput bLED;

	private int redPWMOut = 255;
	private int greenPWMOut = 255;
	private int bluePWMOut = 255;

	private boolean on = false;

	public LEDDriverRGB(DigitalOutput rLED, DigitalOutput gLED, DigitalOutput bLED) {
		this.rLED = rLED;
		this.gLED = gLED;
		this.bLED = bLED;

		this.rLED.setPWMRate(PWM_FREQ);
		this.gLED.setPWMRate(PWM_FREQ);
		this.bLED.setPWMRate(PWM_FREQ);
		this.rLED.enablePWM(0);
		this.gLED.enablePWM(0);
		this.bLED.enablePWM(0);
	}

	public synchronized void set(boolean on) {
		if (on) {
			rLED.updateDutyCycle(redPWMOut/255.0);
			gLED.updateDutyCycle(greenPWMOut/255.0);
			bLED.updateDutyCycle(bluePWMOut/255.0);
		} else {
			rLED.updateDutyCycle(0);
			gLED.updateDutyCycle(0);
			bLED.updateDutyCycle(0);
		}
		this.on = on;
	}

	public synchronized void setLEDColor(RGBColor rgbColor) {
		setLEDColor(rgbColor.red, rgbColor.green, rgbColor.blue);
	}

	public synchronized void setLEDColor(int redPWMOut, int greenPWMOut, int bluePWMOut) {
		this.redPWMOut = redPWMOut;
		this.greenPWMOut = greenPWMOut;
		this.bluePWMOut = bluePWMOut;
		set(on);
	}

}
