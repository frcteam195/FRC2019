package org.aceshigh176.lib.kalman;

import com.github.sh0nk.matplotlib4j.Plot;
import com.github.sh0nk.matplotlib4j.PythonExecutionException;
import com.team254.lib.geometry.Translation2d;
import org.apache.commons.math3.filter.*;
import org.apache.commons.math3.linear.*;
import org.apache.commons.math3.random.JDKRandomGenerator;
import org.apache.commons.math3.random.RandomGenerator;
import org.apache.commons.math3.util.Pair;

import javax.security.sasl.RealmCallback;
import java.io.File;
import java.io.IOException;
import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.stream.Collectors;

public class Kalman {
	// Old stuff
//	{
//		//A - state transition matrix
//		private RealMatrix A;
//		//B - control input matrix
//		private RealMatrix B;
//		//H - measurement matrix
//		private RealMatrix H;
//		//Q - process noise covariance matrix (error in the process)
//		private RealMatrix Q;
//		//R - measurement noise covariance matrix (error in the measurement)
//		private RealMatrix R;
//		//PO - error covariance matrix
//		private RealMatrix PO;
//		//x state
//		private RealVector x;
//
//		// discrete time interval (100ms) between to steps
//		private final double dt = 0.1d;
//		// position measurement noise (10 meter)
//		private final double measurementNoise = 10d;
//		// acceleration noise (meter/sec^2)
//		private final double accelNoise = 0.2d;
//		// constant control input, increase velocity by 0.1 m/s per cycle [vX, vY]
//		private RealVector u = new ArrayRealVector(new double[]{0.1d, 0.1d});
////    private RealVector tmpPNoise = new ArrayRealVector(new double[] { Math.pow(dt, 2d) / 2d, dt });
////    private RealVector mNoise = new ArrayRealVector(1);
//		private KalmanFilter filter;
//
//    public Kalman() {
//		//A and B describe the physic model of the user moving specified as matrices
//		A = new Array2DRowRealMatrix(new double[][]{
//			{1d, 0d, dt, 0d},
//			{0d, 1d, 0d, dt},
//			{0d, 0d, 1d, 0d},
//			{0d, 0d, 0d, 1d}
//		});
//		B = new Array2DRowRealMatrix(new double[][]{
//			{Math.pow(dt, 2d) / 2d},
//			{Math.pow(dt, 2d) / 2d},
//			{dt},
//			{dt}
//		});
//		//only observe first 2 values - the position coordinates
//		H = new Array2DRowRealMatrix(new double[][]{
//			{1d, 0d, 0d, 0d},
//			{0d, 1d, 0d, 0d},
//		});
//		Q = new Array2DRowRealMatrix(new double[][]{
//			{Math.pow(dt, 4d) / 4d, 0d, Math.pow(dt, 3d) / 2d, 0d},
//			{0d, Math.pow(dt, 4d) / 4d, 0d, Math.pow(dt, 3d) / 2d},
//			{Math.pow(dt, 3d) / 2d, 0d, Math.pow(dt, 2d), 0d},
//			{0d, Math.pow(dt, 3d) / 2d, 0d, Math.pow(dt, 2d)}
//		});
//
//		R = new Array2DRowRealMatrix(new double[][]{
//			{Math.pow(measurementNoise, 2d), 0d},
//			{0d, Math.pow(measurementNoise, 2d)}
//		});
//
//        /*PO = new Array2DRowRealMatrix(new double[][] {
//                                                        { 1d, 1d, 1d, 1d },
//                                                        { 1d, 1d, 1d, 1d },
//                                                        { 1d, 1d, 1d, 1d },
//                                                        { 1d, 1d, 1d, 1d }
//                                                     });*/
//
//		// x = [ 0 0 0 0] state consists of position and velocity[pX, pY, vX, vY]
//		//TODO: inititate with map center?
//		x = new ArrayRealVector(new double[]{0, 0, 0, 0});
//
//		ProcessModel pm = new DefaultProcessModel(A, B, Q, x, PO);
//		MeasurementModel mm = new DefaultMeasurementModel(H, R);
//		filter = new KalmanFilter(pm, mm);
//	}
//
//
//		/**
//		 * Use Kalmanfilter to decrease measurement errors
//		 * @param position
//		 * @return
//		 */
//		public Translation2d esimatePosition (Translation2d position){
//		RandomGenerator rand = new JDKRandomGenerator();
//
////        double[] pos = position.toArray();
//		// predict the state estimate one time-step ahead
//		filter.predict(u);
//
////        // noise of the process
////        RealVector  pNoise = tmpPNoise.mapMultiply(accelNoise * pos[0]);
//
//		// x = A * x + B * u + pNoise (state prediction)
////        x = A.operate(x).add(B.operate(u)).add(pNoise);
//		x = A.operate(x).add(B.operate(u));
//
//		// noise of the measurement
////        mNoise.setEntry(0, measurementNoise * rand.nextGaussian());
//
//		// z = H * x + m_noise (measurement prediction)
////        RealVector z = H.operate(x).add(mNoise);
//
//		// correct the state estimate with the latest measurement
//		filter.correct(x);
//
//		//get the corrected state - the position
//		double pX = filter.getStateEstimation()[0];
//		double pY = filter.getStateEstimation()[1];
//
//		return new Translation2d(pX, pY);
//	}
//	}

	// New stuff
//# Estimation parameter of EKF
//	Q = np.diag([1.0, 1.0])**2  # Observation x,y position covariance
//	R = np.diag([0.1, 0.1, np.deg2rad(1.0), 1.0])**2  # predict state covariance
	RealMatrix Q = new Array2DRowRealMatrix(new double[][] {
		{1, 0},
		{0, 1}
	}).power(2);
	RealMatrix R = new Array2DRowRealMatrix(new double[][] {
		{.1, 0, 0, 0},
		{0, .1, 0, 0},
		{0, 0, Math.toRadians(1.0), 0},
		{0, 0, 0, 1.0},
	}).power(2);

//#  Simulation parameter
//	Qsim = np.diag([0.5, 0.5])**2
//	Rsim = np.diag([1.0, np.deg2rad(30.0)])**2
//
	private static final double kDT = 0.1;  // time tick [s]
//	SIM_TIME = 50.0  # simulation time [s]

//	show_animation = False

//	def calc_input():
//	v = 1.0  # [m/s]
//	yawrate = 0.1  # [rad/s]
//	u = np.array([[v, yawrate]]).T
//    return u
	private RealMatrix calc_input() {
		double v = 1.0;
		double yawRate = 0.1;
		return new Array2DRowRealMatrix(new double[] {v, yawRate}).transpose();
	}

//	def observation(xTrue, xd, u):
//
//        xTrue = motion_model(xTrue, u)
//
//        # add noise to gps x-y
//        zx = xTrue[0, 0] + np.random.randn() * Qsim[0, 0]
//        zy = xTrue[1, 0] + np.random.randn() * Qsim[1, 1]
//        z = np.array([[zx, zy]])
//
//        # add noise to input
//        ud1 = u[0, 0] + np.random.randn() * Rsim[0, 0]
//        ud2 = u[1, 0] + np.random.randn() * Rsim[1, 1]
//        ud = np.array([[ud1, ud2]]).T
//
//        xd = motion_model(xd, ud)
//
//        return xTrue, z, xd, ud


//	def motion_model(x, u):
//
//        F = np.array([
//        [1.0, 0, 0, 0],
//            [0, 1.0, 0, 0],
//            [0, 0, 1.0, 0],
//            [0, 0, 0, 0]])
//
//        B = np.array([
//            [DT * math.cos(x[2, 0]), 0],
//            [DT * math.sin(x[2, 0]), 0],
//            [0.0, DT],
//            [1.0, 0.0]])
//
//        x = F.dot(x) + B.dot(u)
//
//		return x

	/**
	 * @param x Previous state
	 * @param u	Robot observation
	 * @return
	 */
	public RealMatrix motion_model(RealMatrix x, RealMatrix u, double DT) {

		// state transition matrix
		RealMatrix F = new Array2DRowRealMatrix(new double[][]{
            {1.0, 0, 0, 0},
            {0, 1.0, 0, 0},
            {0, 0, 1.0, 0},
            {0, 0, 0, 0}
		});

		// Control matrix
		RealMatrix B = new Array2DRowRealMatrix(new double[][]{
			{DT * Math.cos(x.getEntry(2, 0)), 0}, // dx?
			{DT * Math.sin(x.getEntry(2, 0)), 0}, // dy?
			{0, DT},										   // dtheta?
			{1.0, 0},
		});

		x = F.multiply(x).add(B.multiply(u));

		return x;
	}


//	def observation_model(x):
//		#  Observation Model
//        H = np.array([
//            [1, 0, 0, 0],
//            [0, 1, 0, 0]
//            ])
//
//        z = H.dot(x)
//
//		return z

	public RealMatrix observation_model(RealMatrix x) {
		RealMatrix H = new Array2DRowRealMatrix(new double[][]{
			{1.0, 0, 0, 0},
			{0, 1.0, 0, 0},
		});

		var z = H.multiply(x);

		return z;
	}


//	def jacobF(x, u):
//		"""
//        Jacobian of Motion Model
//
//        motion model
//        x_{t+1} = x_t+v*dt*cos(yaw)
//        y_{t+1} = y_t+v*dt*sin(yaw)
//        yaw_{t+1} = yaw_t+omega*dt
//        v_{t+1} = v{t}
//        so
//        dx/dyaw = -v*dt*sin(yaw)
//        dx/dv = dt*cos(yaw)
//        dy/dyaw = v*dt*cos(yaw)
//        dy/dv = dt*sin(yaw)
//        """
//        yaw = x[2, 0]
//        v = u[0, 0]
//        jF = np.array([
//            [1.0, 0.0, -DT * v * math.sin(yaw), DT * math.cos(yaw)],
//            [0.0, 1.0, DT * v * math.cos(yaw), DT * math.sin(yaw)],
//            [0.0, 0.0, 1.0, 0.0],
//            [0.0, 0.0, 0.0, 1.0]])
//
//		return jF

	public RealMatrix jacobF(RealMatrix x, RealMatrix u, double DT) {
        double yaw = x.getEntry(2, 0); // of the system state
        double v = u.getEntry(0, 0); // of the observation
        var jF = new Array2DRowRealMatrix(new double[][] {
			{1.0, 0.0, -DT * v * Math.sin(yaw), DT * Math.cos(yaw)},
            {0.0, 1.0, DT * v * Math.cos(yaw), DT * Math.sin(yaw)},
            {0.0, 0.0, 1.0, 0.0},
            {0.0, 0.0, 0.0, 1.0}});

		return jF;
	}


//	def jacobH(x):
//		# Jacobian of Observation Model
//        jH = np.array([
//            [1, 0, 0, 0],
//            [0, 1, 0, 0]
//            ])
//
//		return jH
	public RealMatrix jacobH(RealMatrix x) {
		RealMatrix jH = new Array2DRowRealMatrix(new double [][] {
			{1.0, 0, 0, 0},
			{0, 1.0, 0, 0},
		});

		return jH;
	}

//	def ekf_estimation(xEst, PEst, z, u):
//
//		#  Predict
//		xPred = motion_model(xEst, u)
//        jF = jacobF(xPred, u)
//        PPred = jF.dot(PEst).dot(jF.T) + R
//
//        #  Update
//		jH = jacobH(xPred)
//        zPred = observation_model(xPred)
//        y = z.T - zPred
//		S = jH.dot(PPred).dot(jH.T) + Q
//        K = PPred.dot(jH.T).dot(np.linalg.inv(S))
//        xEst = xPred + K.dot(y)
//        PEst = (np.eye(len(xEst)) - K.dot(jH)).dot(PPred)
//
//        return xEst, PEst

	/**
	 * @param xEst
	 * @param PEst
	 * @param z gps observation
	 * @param u robot observation
	 * @return
	 */
	public Pair<RealMatrix, RealMatrix> ekf_estimation(RealMatrix xEst, RealMatrix PEst, RealMatrix z, RealMatrix u) {

		// Predict
//		xPred = motion_model(xEst, u)
//      jF = jacobF(xPred, u)
//      PPred = jF.dot(PEst).dot(jF.T) + R
		var xPred = motion_model(xEst, u, kDT);
		var jF = jacobF(xPred, u, kDT);
		var PPred = jF.multiply(PEst).multiply(jF.transpose()).add(R);

		// Update
//		jH = jacobH(xPred)
//      zPred = observation_model(xPred)
//      y = z.T - zPred
//		S = jH.dot(PPred).dot(jH.T) + Q
//      K = PPred.dot(jH.T).dot(np.linalg.inv(S))
//      xEst = xPred + K.dot(y)
//      PEst = (np.eye(len(xEst)) - K.dot(jH)).dot(PPred)
		var jH = jacobH(xPred);
		var zPred = observation_model(xPred);
		var y = z.transpose().subtract(zPred); // Need delta-Z
		var S = jH.multiply(PPred).multiply(jH.transpose()).add(Q);
		var K = PPred.multiply(jH.transpose()).multiply(MatrixUtils.inverse(S));
		xEst = xPred.add(K.multiply(y));
		var k_mul_jh = K.multiply(jH);
		PEst = MatrixUtils.createRealIdentityMatrix(xEst.getRowDimension()).subtract(k_mul_jh).multiply(PPred);

		return new Pair(xEst, PEst);
	}

    public static void main(String[] args) throws IOException, PythonExecutionException {
		HashMap<String, List<String>> csv = CSVtoHashMap.convertCSVFileToHashMap(new File("src/test/java/org/aceshigh176/lib/kalman/fout.csv"));

		ArrayList<Double> gps_x = csv.get("gps_x").stream().map(t -> Double.parseDouble(t)).collect(Collectors.toCollection(ArrayList::new));
		ArrayList<Double> gps_y = csv.get("gps_y").stream().map(t -> Double.parseDouble(t)).collect(Collectors.toCollection(ArrayList::new));
		ArrayList<Double> true_x = csv.get("true_x").stream().map(t -> Double.parseDouble(t)).collect(Collectors.toCollection(ArrayList::new));
		ArrayList<Double> true_y = csv.get("true_y").stream().map(t -> Double.parseDouble(t)).collect(Collectors.toCollection(ArrayList::new));
		ArrayList<Double> dead_reckon_x = csv.get("dead_reckon_x").stream().map(t -> Double.parseDouble(t)).collect(Collectors.toCollection(ArrayList::new));
		ArrayList<Double> dead_reckon_y = csv.get("dead_reckon_y").stream().map(t -> Double.parseDouble(t)).collect(Collectors.toCollection(ArrayList::new));
		ArrayList<Double> u_v = csv.get("u_v").stream().map(t -> Double.parseDouble(t)).collect(Collectors.toCollection(ArrayList::new));
		ArrayList<Double> u_yawrate = csv.get("u_yawrate").stream().map(t -> Double.parseDouble(t)).collect(Collectors.toCollection(ArrayList::new));
		ArrayList<Double> estimated_x = csv.get("estimated_x").stream().map(t -> Double.parseDouble(t)).collect(Collectors.toCollection(ArrayList::new));
		ArrayList<Double> estimated_y = csv.get("estimated_y").stream().map(t -> Double.parseDouble(t)).collect(Collectors.toCollection(ArrayList::new));

		ArrayList<Double> java_est_x = new ArrayList<>();
		ArrayList<Double> java_est_y = new ArrayList<>();


//        # State Vector [x y yaw v]'
//		xEst = np.zeros((4, 1))
//		xTrue = np.zeros((4, 1))
//		PEst = np.eye(4)
//
//		xDR = np.zeros((4, 1))  # Dead reckoning

		Kalman kalman = new Kalman();

		RealMatrix xEst = MatrixUtils.createRealMatrix(4, 1);
//		RealMatrix xTrue = MatrixUtils.createRealMatrix(4, 1);
		RealMatrix PEst = MatrixUtils.createRealIdentityMatrix(4);


		for(int i = 0; i < gps_x.size(); i++) {
//			u = calc_input()
//
//			xTrue, z, xDR, ud = observation(xTrue, xDR, u)

//			xTrue =
			var z = MatrixUtils.createRowRealMatrix(new double[]{gps_x.get(i), gps_y.get(i)});
			var ud = MatrixUtils.createColumnRealMatrix(new double[]{u_v.get(i), u_yawrate.get(i)});

			var estimation = kalman.ekf_estimation(xEst, PEst, z, ud);
			xEst = estimation.getFirst();
			PEst = estimation.getSecond();

			java_est_x.add(xEst.getEntry(0, 0));
			java_est_y.add(xEst.getEntry(1, 0));
			System.out.println(xEst.getEntry(0, 0) + ", " + xEst.getEntry(1, 0));
		}

		Plot plt = Plot.create();
//		plt.cla();
		plt.plot()
			.add(gps_x, gps_y, ".g") // GPS samples
			.add(true_x, true_y, "-b") // True path
			.add(dead_reckon_x, dead_reckon_y, "-k") // Dead reckoning
			.add(java_est_x, java_est_y, "-r") // Estimated path
//			.add(estimated_x, estimated_y, "-r") // Estimated path
		;
//		plot_covariance_ellipse(xEst, PEst) # covariance ellipse
//		plt.axis("equal");
//		plt.grid(True);
//		plt.pause(0.001);
		plt.show();
	}
}