package com.team195.lib.drivers;

public interface CKIMU {

	double getFusedHeading();

	boolean isPresent();

	boolean reset();
}
