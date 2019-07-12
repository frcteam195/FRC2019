package com.team195.lib.util.statefuser;

import org.apache.commons.math3.filter.MeasurementModel;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;

/**
 * See @{@link org.apache.commons.math3.filter.DefaultMeasurementModel} for any idea of what the heck is going on here
 * ¯\_(ツ)_/¯
 */
public class CKSingleAxisNoUnitTransformMeasurementModel implements MeasurementModel {
	private static final Array2DRowRealMatrix measurementMatrixNoTransform = new Array2DRowRealMatrix(new double[][] {new double[] {1, 0}, new double[] {0, 1}});

	private RealMatrix measurementNoise;

	public CKSingleAxisNoUnitTransformMeasurementModel() {

	}

	public CKSingleAxisNoUnitTransformMeasurementModel(RealMatrix sensorMeasurementNoise) {
		this.measurementNoise = sensorMeasurementNoise;
	}

	@Override
	public RealMatrix getMeasurementMatrix() {
		return measurementMatrixNoTransform;
	}

	//This is based on the noise of the sensor measurement
	@Override
	public RealMatrix getMeasurementNoise() {
		return measurementNoise;
	}

	public void setMeasurementNoiseMatrix(MeasurementModel sensorMeasurementModel) {
		measurementNoise = sensorMeasurementModel.getMeasurementNoise();
	}

	public void setMeasurementNoiseMatrix(RealMatrix sensorMeasurementNoise) {
		this.measurementNoise = sensorMeasurementNoise;
	}
}
