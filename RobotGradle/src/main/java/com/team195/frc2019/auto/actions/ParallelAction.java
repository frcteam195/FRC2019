package com.team195.frc2019.auto.actions;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.reporters.MessageLevel;

/**
 * Composite action, running all sub-actions at the same time. All actions are started then periodically updated until all actions
 * report being done.
 */
public class ParallelAction implements Action {

    private final ArrayList<Action> mActions;

    public ParallelAction(List<Action> actions) {
        mActions = new ArrayList<>(actions);
    }

    public ParallelAction(Action... actions) {
        this(Arrays.asList(actions));
    }

    @Override
    public boolean isFinished() {
        for (Action action : mActions) {
            if (!action.isFinished()) {
                return false;
            }
        }
        return true;
    }

    @Override
    public void update() {
        for (Action action : mActions) {
            action.update();
        }
    }

    @Override
    public void done() {
        for (Action action : mActions) {
//            ConsoleReporter.report("Finishing: " + action.getClass().getSimpleName(), MessageLevel.INFO);
            action.done();
        }
    }

    @Override
    public void start() {
        for (Action action : mActions) {
//            ConsoleReporter.report("Starting : " + action.getClass().getSimpleName(), MessageLevel.INFO);
            action.start();
        }
    }
}
