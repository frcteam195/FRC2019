package com.team195.frc2019;

import com.illposed.osc.OSCListener;
import com.illposed.osc.OSCPortIn;
import com.team195.frc2019.auto.AutoModeBase;
import com.team195.frc2019.auto.modes.CharacterizeHighGearStraight;
import com.team195.frc2019.auto.modes.CrossAutoLineMode;
import com.team195.frc2019.auto.modes.DesiredMode;
import com.team195.frc2019.auto.modes.DoNothingMode;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.lib.util.StartingPosition;

import java.util.List;

public class AutoModeSelector {
    private DesiredMode mCachedDesiredMode = null;
    private StartingPosition mCachedStartingPosition = null;
    private OSCPortIn oscPortIn;

    public AutoModeSelector() {
        try {
            oscPortIn = new OSCPortIn(Constants.LOG_OSC_REPORTER_PORT);

            OSCListener autoSelectionListener = (time, message) -> {
                try {
                    List<Object> args = message.getArguments();
                    if (args.size() > 1) {
                        StartingPosition tmpStartingPosition = StartingPosition.valueOf((int)args.get(0));
                        if (tmpStartingPosition != StartingPosition.Invalid)
                            mCachedStartingPosition = tmpStartingPosition;
                        DesiredMode tmpDesiredMode = DesiredMode.valueOf((int)args.get(1));
                        if (tmpDesiredMode != DesiredMode.Invalid)
                            mCachedDesiredMode = tmpDesiredMode;
                    }
                } catch (Exception ex) {
                    ex.printStackTrace();
                }
            };
            oscPortIn.addListener("/AutoData", autoSelectionListener);



            oscPortIn.startListening();
        } catch (Exception ex) {
            ConsoleReporter.report(ex);
        }
    }

    public void reset() {
        mCachedDesiredMode = null;
        mCachedStartingPosition = null;
    }

    public AutoModeBase getAutoMode() {
        if (mCachedDesiredMode != null && mCachedStartingPosition != null) {
            switch (mCachedDesiredMode) {
                case CrossAutoLine:
                    switch (mCachedStartingPosition) {
                        case Left:
                            break;
                        case Center:
                            break;
                        case Right:
                            break;
                        case Invalid:
                            break;
                    }
                    return new CrossAutoLineMode();
                case Characterization:
                    return new CharacterizeHighGearStraight();
                case DoNothing:
                    return new DoNothingMode();
                default:
                    return new CrossAutoLineMode();
            }
        }
        return new DoNothingMode();
    }

}
