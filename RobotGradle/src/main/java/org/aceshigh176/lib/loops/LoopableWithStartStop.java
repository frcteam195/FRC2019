package org.aceshigh176.lib.loops;

public interface LoopableWithStartStop extends Loopable {

    void onStart(double timestamp);

    void onStop(double timestamp);

}
