package com.team195.frc2019;

import com.illposed.osc.OSCListener;
import com.illposed.osc.OSCPortIn;
import com.team195.frc2019.auto.AutoModeBase;
import com.team195.frc2019.auto.modes.*;
import com.team195.frc2019.auto.modes.High.HighTwoHatchCargoshipBackwardsMode;
import com.team195.frc2019.auto.modes.Low.LowTwoHatchCargoshipBackwardsMode;
import com.team195.frc2019.constants.Constants;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.lib.util.StartingPosition;

import java.util.List;

public class AutoModeSelector {
    private DesiredMode mCachedDesiredMode = null;
    private StartingPosition mCachedStartingPosition = null;
    private OSCPortIn oscPortIn;

    public AutoModeSelector() {
        try {
            oscPortIn = new OSCPortIn(Constants.AUTO_SELECTOR_PORT);

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
            switch (mCachedStartingPosition) {
                case LeftLow:
                    switch (mCachedDesiredMode) {
                        case TwoHatchRocket:
                            return new DoNothingMode();
                        case TwoHatchCargoship:
                            return new LowTwoHatchCargoshipBackwardsMode(true);
                        default:
                            break;
                    }
                    break;
                case LeftHigh:
                    switch (mCachedDesiredMode) {
                        case TwoHatchRocket:
                            return new DoNothingMode();
                        case TwoHatchCargoship:
                            return new HighTwoHatchCargoshipBackwardsMode(true);
                        default:
                            break;
                    }
                    break;
                case Center:
                    return new TestMode();
                case RightLow:
                    switch (mCachedDesiredMode) {
                        case TwoHatchRocket:
                            return new DoNothingMode();
                        case TwoHatchCargoship:
                            return new LowTwoHatchCargoshipBackwardsMode(false);
                        default:
                            break;
                    }
                    break;
                case RightHigh:
                    switch (mCachedDesiredMode) {
                        case TwoHatchRocket:
                            return new DoNothingMode();
                        case TwoHatchCargoship:
                            return new HighTwoHatchCargoshipBackwardsMode(false);
                        default:
                            break;
                    }
                    break;
                case Invalid:
                    break;
            }
        }
        return new DoNothingMode();
    }

}
