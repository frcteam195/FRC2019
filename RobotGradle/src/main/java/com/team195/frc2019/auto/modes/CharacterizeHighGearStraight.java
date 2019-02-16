package com.team195.frc2019.auto.modes;

import com.team195.frc2019.auto.AutoModeEndedException;
import com.team195.frc2019.auto.AutoModeBase;
import com.team195.frc2019.auto.actions.CollectAccelerationData;
import com.team195.frc2019.auto.actions.CollectVelocityData;
import com.team195.frc2019.auto.actions.WaitAction;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team254.lib.physics.DriveCharacterization;

import java.util.ArrayList;
import java.util.List;

public class CharacterizeHighGearStraight extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        List<DriveCharacterization.VelocityDataPoint> velocityData = new ArrayList<>();
        List<DriveCharacterization.AccelerationDataPoint> accelerationData = new ArrayList<>();

        // runAction(new ShiftHighGearAction(false));
        // runAction(new WaitAction(10));

        runAction(new CollectVelocityData(velocityData, false, false, true));
        runAction(new WaitAction(10));
        runAction(new CollectAccelerationData(accelerationData, false, false, true));

        DriveCharacterization.CharacterizationConstants constants = DriveCharacterization.characterizeDrive(velocityData, accelerationData);

        ConsoleReporter.report("ks: " + constants.ks);
        ConsoleReporter.report("kv: " + constants.kv);
        ConsoleReporter.report("ka: " + constants.ka);
    }
}
