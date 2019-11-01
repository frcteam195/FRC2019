package com.team195.lib.drivers;

public class RealsenseJNI {

	public native int getTranslation(float[] translationContainer);
	public native int getRotation(float[] rotationContainer);
	public native int setTranslation(float[] startingPosition);
	public native int setRotation(float[] startingRotation);

	static boolean libraryLoaded = false;

	static {
		if (!libraryLoaded) {
			try {
				System.loadLibrary("ToddLib_JNI");
			} catch (UnsatisfiedLinkError e) {
				e.printStackTrace();
				System.exit(1);
			}
			libraryLoaded = true;
		}
	}
}
