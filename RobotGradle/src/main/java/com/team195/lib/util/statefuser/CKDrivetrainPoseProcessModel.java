package com.team195.lib.util.statefuser;

import com.team195.frc2019.Robot;
import com.team195.frc2019.auto.AutoModeBase;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;

public class CKDrivetrainPoseProcessModel {

	private CKSingleAxisProcessModel xCoordProcessModel = new CKSingleAxisProcessModel(new Array2DRowRealMatrix(new double[][] {new double[]{1, 0.85},
																																new double[]{0.85, 1}}));

	private CKSingleAxisProcessModel yCoordProcessModel = new CKSingleAxisProcessModel(new Array2DRowRealMatrix(new double[][] {new double[]{1, 0.85},
																																new double[]{0.85, 1}}));

	private CKSingleAxisProcessModel rotCoordProcessModel = new CKSingleAxisProcessModel(new Array2DRowRealMatrix(new double[][] {new double[]{1, 0.85},
																																  new double[]{0.85, 1}}));

	private AutoModeBase autoMode;

	public CKDrivetrainPoseProcessModel() {

	}

	public void update() {
		autoMode = Robot.mAutoModeExecutor.getAutoMode();
		if (autoMode != null) {
			xCoordProcessModel.update(autoMode.getStartingCoords().getTranslation().x());
			yCoordProcessModel.update(autoMode.getStartingCoords().getTranslation().y());
			rotCoordProcessModel.update(autoMode.getStartingCoords().getRotation().getRadians());
		}
		else {
			xCoordProcessModel.update(0);
			yCoordProcessModel.update(0);
			rotCoordProcessModel.update(0);
		}
	}

	public CKSingleAxisProcessModel getxCoordProcessModel() {
		return xCoordProcessModel;
	}

	public CKSingleAxisProcessModel getyCoordProcessModel() {
		return yCoordProcessModel;
	}

	public CKSingleAxisProcessModel getRotCoordProcessModel() {
		return rotCoordProcessModel;
	}
}
