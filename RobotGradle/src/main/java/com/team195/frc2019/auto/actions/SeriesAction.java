package com.team195.frc2019.auto.actions;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Executes one action at a time. Useful as a member of {@link ParallelAction}
 */
public class SeriesAction implements Action {

    private int mCurActionIndex;
    private Action mCurAction;
    private final ArrayList<Action> mActions;

    public SeriesAction(List<Action> actions) {
        mActions = new ArrayList<>(actions);
        mCurActionIndex = 0;
        mCurAction = null;
    }

    public SeriesAction(Action... actions) {
        this(Arrays.asList(actions));
    }

    @Override
    public boolean isFinished() {
        return mCurAction == null && mCurActionIndex == mActions.size();
    }

    @Override
    public void start() {
    	mCurAction = null;
    	mCurActionIndex = 0;
    }

    @Override
    public void update() {
        if (mCurAction == null) {
            if (mCurActionIndex >= mActions.size()) {
                return;
            }

            mCurAction = mActions.get(mCurActionIndex++);
//            ConsoleReporter.report("Starting : " + mCurAction.getClass().getSimpleName(), MessageLevel.INFO);
            mCurAction.start();
        }

        mCurAction.update();

        if (mCurAction.isFinished()) {
//            ConsoleReporter.report("Finishing: " + mCurAction.getClass().getSimpleName(), MessageLevel.INFO);
            mCurAction.done();
            mCurAction = null;
        }
    }

    public void purgeActions() {
	    if(mCurAction == null) {
		    if(mCurActionIndex >= mActions.size()) {
			    return;
		    }
	    }
	    if(mCurAction != null) {
		    mCurAction.done();
	    }
	    for(Action a :
			    mActions) {
		    if(a != null) {
			    a.start();
			    a.update();
			    a.isFinished();
			    a.done();
		    }
	    }
	    mCurAction = null;
	    mCurActionIndex = mActions.size();
    }

    @Override
    public void done() {
    }
}
