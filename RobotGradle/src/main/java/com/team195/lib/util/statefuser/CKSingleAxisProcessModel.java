package com.team195.lib.util.statefuser;

import com.team195.lib.util.ElapsedTimer;
import org.apache.commons.math3.filter.ProcessModel;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

/**
 * See @{@link org.apache.commons.math3.filter.DefaultProcessModel} for any idea of what the heck is going on here
 * ¯\_(ツ)_/¯
 */
public class CKSingleAxisProcessModel implements ProcessModel {

	/**
	 * The state transition matrix, used to advance the internal state estimation each time-step.
	 */
	private RealMatrix stateTransitionMatrix;

	/**
	 * The control matrix, used to integrate a control input into the state estimation.
	 */
	private RealMatrix controlMatrix;

	/** The process noise covariance matrix. */
	private RealMatrix processNoiseCovMatrix;

	/** The initial state estimation of the observed process. */
	private RealVector initialStateEstimateVector;

	/** The initial error covariance matrix of the observed process. */
	private RealMatrix initialErrorCovMatrix;

	private final ElapsedTimer mRealDtTimer = new ElapsedTimer();
	private double dt = 0;

	public CKSingleAxisProcessModel(RealMatrix initialErrorCovMatrix) {
		mRealDtTimer.start();
		//Fk
		this.stateTransitionMatrix = new Array2DRowRealMatrix(new double[][] {new double[]{1, 0},
																			  new double[]{0, 1}});

		//Bk
		this.controlMatrix = new Array2DRowRealMatrix(new double[][] {new double[]{0},
																	  new double[]{0}});

		//Qk
		this.processNoiseCovMatrix = new Array2DRowRealMatrix(new double[][] {new double[]{0, 0},
																			  new double[]{0, 0}});

		//X0
		this.initialStateEstimateVector = new ArrayRealVector(new double[] {0, 0});

		//initial error covariance
		//Pk
		//|1    0.85|
		//|0.85    1|
		//		this.initialErrorCovMatrix = new Array2DRowRealMatrix(new double[][] {new double[]{1, 0.85},
		//                                                              			new double[]{0.85, 1}});
		this.initialErrorCovMatrix = initialErrorCovMatrix;

		update(0);
	}

	public void update(double initialPos) {
		dt = mRealDtTimer.hasElapsed();

		/*
		|1, dt |
		|0, 1  |
		 */
		stateTransitionMatrix.setEntry(0, 1, dt);

		/*
		|(dt^2)/2   |
		|dt         |
		 */
		controlMatrix.setEntry(0, 0, Math.pow(dt, 2)/2.0);
		controlMatrix.setEntry(1, 0, dt);

		/*
		|pos|
		|vel|
		 */
		initialStateEstimateVector.setEntry(0, initialPos);

		mRealDtTimer.start();
	}

	@Override
	public RealMatrix getStateTransitionMatrix() {
		return stateTransitionMatrix;
	}

	@Override
	public RealMatrix getControlMatrix() {
		return controlMatrix;
	}

	@Override
	public RealMatrix getProcessNoise() {
		return processNoiseCovMatrix;
	}

	@Override
	public RealVector getInitialStateEstimate() {
		return initialStateEstimateVector;
	}

	@Override
	public RealMatrix getInitialErrorCovariance() {
		return initialErrorCovMatrix;
	}
}