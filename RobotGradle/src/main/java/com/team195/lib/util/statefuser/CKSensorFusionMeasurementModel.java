package com.team195.lib.util.statefuser;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;

public class CKSensorFusionMeasurementModel {
	private static final RealMatrix currOdometryMeasurementNoiseMatrix = new Array2DRowRealMatrix(new double[][] {new double[] {1, 0.70},
																								                  new double[] {0.70, 1}});
	private static CKSingleAxisNoUnitTransformMeasurementModel odometryMeasurementModel = new CKSingleAxisNoUnitTransformMeasurementModel(currOdometryMeasurementNoiseMatrix);

	private static CKSingleAxisNoUnitTransformMeasurementModel cameraMeasurementModel = new CKSingleAxisNoUnitTransformMeasurementModel();

	public static void update(byte cameraConfidence) {
		cameraMeasurementModel.setMeasurementNoiseMatrix(CKRealsenseMatrices.getConfidenceMeasurementModelMatrix(cameraConfidence));
	}

	public static CKSingleAxisNoUnitTransformMeasurementModel getOdometryMeasurementModel() {
		return odometryMeasurementModel;
	}

	public static CKSingleAxisNoUnitTransformMeasurementModel getCameraMeasurementModel() {
		return cameraMeasurementModel;
	}
}
