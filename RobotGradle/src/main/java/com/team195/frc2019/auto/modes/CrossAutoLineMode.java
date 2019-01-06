package com.team195.frc2019.auto.modes;

import com.team195.frc2019.auto.AutoModeEndedException;
import com.team195.frc2019.auto.AutoModeBase;
import com.team195.frc2019.auto.actions.OpenLoopDrive;
import com.team195.frc2019.auto.actions.WaitAction;

public class CrossAutoLineMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Cross auto line");
        runAction(new WaitAction(5.0));
        runAction(new OpenLoopDrive(-0.3, -0.3, 5.0, false));
    }
}
