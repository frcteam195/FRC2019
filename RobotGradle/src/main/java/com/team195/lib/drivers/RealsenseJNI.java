package com.team195.lib.drivers;

public class RealsenseJNI {

	public static native int getTranslation(float[] translationContainer);
	public static native int getRotation(float[] rotationContainer);
	public static native int setTranslation(float[] startingPosition);
	public static native int setRotation(float[] startingRotation);

	static boolean libraryLoaded = false;

	static {
		if (!libraryLoaded) {
			try {
				System.loadLibrary("RealsenseJNI");
			} catch (UnsatisfiedLinkError e) {
				e.printStackTrace();
				System.exit(1);
			}
			libraryLoaded = true;
		}
	}
}
