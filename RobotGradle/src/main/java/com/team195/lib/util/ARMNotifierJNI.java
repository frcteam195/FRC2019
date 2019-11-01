package com.team195.lib.util;

public class ARMNotifierJNI {
    public static native int sleepForNano(int nano);

    public static native int setCANInterface_BobAndTodd(String canInterface);

    static boolean libraryLoaded = false;

    static {
        if (!libraryLoaded) {
            try {
                System.loadLibrary("JNIARMNotifier");
            } catch (UnsatisfiedLinkError e) {
                e.printStackTrace();
                System.exit(1);
            }
            libraryLoaded = true;
        }
    }
}