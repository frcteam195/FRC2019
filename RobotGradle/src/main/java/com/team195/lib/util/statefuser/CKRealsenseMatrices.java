package com.team195.lib.util.statefuser;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;

public class CKRealsenseMatrices {
	public static final RealMatrix COV_HIGH_CONFIDENCE = new Array2DRowRealMatrix(new double[][] {new double[]{1, 1},
																								  new double[]{1, 1}});

	public static final RealMatrix COV_MED_CONFIDENCE = new Array2DRowRealMatrix(new double[][] {new double[]{1, 0.92},
																								 new double[]{0.92, 1}});

	public static final RealMatrix COV_LOW_CONFIDENCE = new Array2DRowRealMatrix(new double[][] {new double[]{1, 0.85},
																								 new double[]{0.85, 1}});

	public static final RealMatrix COV_NO_CONFIDENCE = new Array2DRowRealMatrix(new double[][] {new double[]{0, 0},
																								new double[]{0, 0}});

	public static RealMatrix getConfidenceMeasurementModelMatrix(byte confidence) {
		switch (confidence) {
			case 0x00:
				return COV_NO_CONFIDENCE;
			case 0x01:
				return COV_LOW_CONFIDENCE;
			case 0x02:
				return COV_MED_CONFIDENCE;
			case 0x03:
				return COV_HIGH_CONFIDENCE;
			default:
				return COV_NO_CONFIDENCE;
		}
	}
}
