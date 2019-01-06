package com.team195.lib.util.drivers;


import com.team195.lib.util.RGBColor;

public interface LEDDriver {
	void set(boolean on);
	void setLEDColor(RGBColor rgbColor);
}
