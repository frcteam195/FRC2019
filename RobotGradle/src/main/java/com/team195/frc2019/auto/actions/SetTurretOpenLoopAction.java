package com.team195.frc2019.auto.actions;

import com.team195.frc2019.subsystems.Turret;
import com.team195.frc2019.subsystems.positions.TurretPositions;

import java.util.function.Function;

public class SetTurretOpenLoopAction implements Action {
	private static final Turret mTurret = Turret.getInstance();

	Function<Void, Boolean> mButtonGetterMethod;
	Function<Void, Double> mAxisGetterMethod;

	public SetTurretOpenLoopAction(Function<Void, Boolean> buttonGetterMethod, Function<Void, Double> axisGetterMethod) {
		mButtonGetterMethod = buttonGetterMethod;
		mAxisGetterMethod = axisGetterMethod;
	}

	@Override
	public boolean isFinished() {
		return (!mButtonGetterMethod.apply(null));
	}

	@Override
	public void update() {
		mTurret.setTurretPosition(mAxisGetterMethod.apply(null) / 3.0);
	}

	@Override
	public void done() {
		mTurret.setTurretControlMode(Turret.TurretControlMode.DISABLED);
	}

	@Override
	public void start() {
		mTurret.setTurretControlMode(Turret.TurretControlMode.OPEN_LOOP);
	}
}
