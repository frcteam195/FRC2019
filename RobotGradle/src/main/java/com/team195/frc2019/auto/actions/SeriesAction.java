package com.team195.frc2019.auto.actions;

import java.util.ArrayList;
import java.util.List;

import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.reporters.MessageLevel;

/**
 * Executes one action at a time. Useful as a member of {@link ParallelAction}
 */
public class SeriesAction implements Action {

    private Action mCurAction;
    private final ArrayList<Action> mRemainingActions;

    public SeriesAction(List<Action> actions) {
        mRemainingActions = new ArrayList<>(actions);
        mCurAction = null;
    }

    @Override
    public boolean isFinished() {
        return mRemainingActions.isEmpty() && mCurAction == null;
    }

    @Override
    public void start() {
    }

    @Override
    public void update() {
        if (mCurAction == null) {
            if (mRemainingActions.isEmpty()) {
                return;
            }

            mCurAction = mRemainingActions.remove(0);
            ConsoleReporter.report("Starting : " + mCurAction.getClass().getSimpleName(), MessageLevel.INFO);
            mCurAction.start();
        }

        mCurAction.update();

        if (mCurAction.isFinished()) {
            ConsoleReporter.report("Finishing: " + mCurAction.getClass().getSimpleName(), MessageLevel.INFO);
            mCurAction.done();
            mCurAction = null;
        }
    }

    @Override
    public void done() {
    }
}
