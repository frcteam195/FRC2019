package com.team195.frc2019.auto.creators;

import com.team195.frc2019.auto.modes.CrossAutoLineMode;
import com.team195.frc2019.AutoFieldState;
import com.team195.frc2019.auto.AutoModeBase;

public class CrossAutoLineCreator implements AutoModeCreator {

    // Pre-build trajectories to go left and right
    private CrossAutoLineMode mCrossAutoLineMode = new CrossAutoLineMode();

    @Override
    public AutoModeBase getStateDependentAutoMode(AutoFieldState fieldState) {
        return mCrossAutoLineMode;
    }
}
