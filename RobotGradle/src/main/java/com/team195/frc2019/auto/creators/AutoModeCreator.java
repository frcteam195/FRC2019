package com.team195.frc2019.auto.creators;

import com.team195.frc2019.AutoFieldState;
import com.team195.frc2019.auto.AutoModeBase;

public interface AutoModeCreator {
    AutoModeBase getStateDependentAutoMode(AutoFieldState fieldState);
}
