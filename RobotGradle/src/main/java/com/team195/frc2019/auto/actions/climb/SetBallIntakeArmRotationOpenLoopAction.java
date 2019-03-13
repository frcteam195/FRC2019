package com.team195.frc2019.auto.actions.climb;

import com.team195.frc2019.auto.actions.Action;
import com.team195.frc2019.subsystems.BallIntakeArm;

import java.util.function.Function;

public class SetBallIntakeArmRotationOpenLoopAction implements Action {
	private static final BallIntakeArm mBallIntakeArm = BallIntakeArm.getInstance();

	private final Function<Void, Boolean> mButtonGetterMethod;

	private final double mOutput;

	public SetBallIntakeArmRotationOpenLoopAction(double output, Function<Void, Boolean> buttonGetterMethod) {
		mButtonGetterMethod = buttonGetterMethod;
		mOutput = output;
	}

	@Override
	public boolean isFinished() {
		return (!mButtonGetterMethod.apply(null));
	}

	@Override
	public void update() {  }

	@Override
	public void done() {
		mBallIntakeArm.setBallIntakeArmPosition(0);
	}

	@Override
	public void start() {
		mBallIntakeArm.setBallIntakeArmControlMode(BallIntakeArm.BallIntakeArmControlMode.OPEN_LOOP);
		mBallIntakeArm.setBallIntakeArmPosition(mOutput);
	}
}
