package com.team195.lib.util.statefuser;

import com.team195.frc2019.RobotState;
import com.team195.frc2019.SensorFusedRobotState;
import com.team195.frc2019.subsystems.Drive;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import org.apache.commons.math3.filter.KalmanFilter;
import org.apache.commons.math3.filter.ProcessModel;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

public class StateEstimationFuser {
	//uk = Control Vector - Acceleration in 2d
	//| a |
	//| 0 |

	private static final RobotState mRobotState = RobotState.getInstance();
	private static final SensorFusedRobotState mSensorFusedRobotState = SensorFusedRobotState.getInstance();
	private static final Drive mDrive = Drive.getInstance();

	private CKDrivetrainPoseProcessModel mProcessModel = new CKDrivetrainPoseProcessModel();

	private Pose2d currOdometryPose;
	private Pose2d currFusedPose;

	private Pose2d currCameraPose = new Pose2d();
	private Velocity3d currCameraLinearVel = new Velocity3d(0,0,0);
	private Velocity3d currCameraAngularVel = new Velocity3d(0,0,0);
	private byte currCameraConfidence;
	private Quaternion currCameraRotation = new Quaternion(0, 0, 0, 0);

	private ProcessModel m_X_pm = mProcessModel.getxCoordProcessModel();
	private CKSingleAxisNoUnitTransformMeasurementModel m_X_mm = new CKSingleAxisNoUnitTransformMeasurementModel();
	private KalmanFilter m_X_kf = new KalmanFilter(m_X_pm, m_X_mm);
	private RealVector m_X_ctrlVector_u = new ArrayRealVector(new double[] {0, 0}); //[accel, 0]
	private RealVector m_X_measurementVector_z = new ArrayRealVector(new double[] {0, 0});  //Pos, Vel

	private ProcessModel m_Y_pm = mProcessModel.getyCoordProcessModel();
	private CKSingleAxisNoUnitTransformMeasurementModel m_Y_mm = new CKSingleAxisNoUnitTransformMeasurementModel();
	private KalmanFilter m_Y_kf = new KalmanFilter(m_Y_pm, m_Y_mm);
	private RealVector m_Y_ctrlVector_u = new ArrayRealVector(new double[] {0, 0});
	private RealVector m_Y_measurementVector_z = new ArrayRealVector(new double[] {0, 0});

	private ProcessModel m_R_pm = mProcessModel.getRotCoordProcessModel();
	private CKSingleAxisNoUnitTransformMeasurementModel m_R_mm = new CKSingleAxisNoUnitTransformMeasurementModel();
	private KalmanFilter m_R_kf = new KalmanFilter(m_R_pm, m_R_mm);
	private RealVector m_R_ctrlVector_u = new ArrayRealVector(new double[] {0, 0});
	private RealVector m_R_measurementVector_z = new ArrayRealVector(new double[] {0, 0});

	private double currOdometryAngle = 0;
	private double currFusedPoseAngle = 0;

	public StateEstimationFuser() {
	}

	public void update() {
		currOdometryPose = mRobotState.getLatestFieldToVehicle().getValue();
		currOdometryAngle = currOdometryPose.getRotation().getRadians();

		currFusedPose = mSensorFusedRobotState.getLatestFieldToVehicle().getValue();
		currFusedPoseAngle = currFusedPose.getRotation().getRadians();


		currCameraPose = null; //TODO: Get Pose from camera
		currCameraConfidence = 0; //TODO: Get Confidence from camera
		currCameraLinearVel.set(0, 0, 0); //TODO: Get Linear Velocity from camera
		currCameraAngularVel.set(0, 0, 0); //TODO: Get Angular Velocity from camera
		currCameraRotation.set(0, 0, 0, 0); //TODO: Get Rotation from Camera

		CKSensorFusionMeasurementModel.update(currCameraConfidence);
		mProcessModel.update();

		//x accel matrix
		m_X_ctrlVector_u.setEntry(0, Math.cos(currFusedPoseAngle) * mDrive.getChassisAccel().linear);
		m_X_kf.predict(m_X_ctrlVector_u);
		m_X_mm.setMeasurementNoiseMatrix(CKSensorFusionMeasurementModel.getOdometryMeasurementModel());
		m_X_measurementVector_z.setEntry(0, currOdometryPose.getTranslation().x());
		m_X_measurementVector_z.setEntry(1, Math.cos(currOdometryAngle) * mDrive.getLinearVelocity());
		m_X_kf.correct(m_X_measurementVector_z);
		m_X_mm.setMeasurementNoiseMatrix(CKSensorFusionMeasurementModel.getCameraMeasurementModel());
		m_X_measurementVector_z.setEntry(0, currCameraPose.getTranslation().x());
		m_X_measurementVector_z.setEntry(1, currCameraLinearVel.x());
		m_X_kf.correct(m_X_measurementVector_z);

		//y accel matrix
		m_Y_ctrlVector_u.setEntry(0, Math.sin(currFusedPoseAngle) * mDrive.getChassisAccel().linear);
		m_Y_kf.predict(m_Y_ctrlVector_u);
		m_Y_mm.setMeasurementNoiseMatrix(CKSensorFusionMeasurementModel.getOdometryMeasurementModel());
		m_Y_measurementVector_z.setEntry(0, currOdometryPose.getTranslation().y());
		m_Y_measurementVector_z.setEntry(1, Math.sin(currOdometryAngle) * mDrive.getLinearVelocity());
		m_Y_kf.correct(m_Y_measurementVector_z);
		m_Y_mm.setMeasurementNoiseMatrix(CKSensorFusionMeasurementModel.getCameraMeasurementModel());
		m_Y_measurementVector_z.setEntry(0, currCameraPose.getTranslation().y());
		m_Y_measurementVector_z.setEntry(1, currCameraLinearVel.y());
		m_Y_kf.correct(m_Y_measurementVector_z);

		//angular accel matrix
		m_R_ctrlVector_u.setEntry(0, mDrive.getChassisAccel().angular);
		m_R_kf.predict(m_R_ctrlVector_u);
		m_R_mm.setMeasurementNoiseMatrix(CKSensorFusionMeasurementModel.getOdometryMeasurementModel());
		m_R_measurementVector_z.setEntry(0, currOdometryPose.getRotation().getRadians());
		m_R_measurementVector_z.setEntry(1, mDrive.getAngularVelocity());
		m_R_kf.correct(m_R_measurementVector_z);
		m_R_mm.setMeasurementNoiseMatrix(CKSensorFusionMeasurementModel.getCameraMeasurementModel());
		m_R_measurementVector_z.setEntry(0, RotationAdvanced2d.fromQuaternionYaw(currCameraRotation).getRadians());
		m_R_measurementVector_z.setEntry(1, currCameraAngularVel.x());
		m_R_kf.correct(m_R_measurementVector_z);

		mSensorFusedRobotState.addFieldToVehicleObservation(Timer.getFPGATimestamp(),
				new Pose2d(m_X_kf.getStateEstimationVector().getEntry(0),
						m_Y_kf.getStateEstimationVector().getEntry(0),
						Rotation2d.fromRadians(m_R_kf.getStateEstimationVector().getEntry(0))));
	}
}
