package com.team195.frc2019.subsystems;

import com.team195.frc2019.constants.TargetingConstants;
import com.team195.frc2019.loops.ILooper;
import com.team195.frc2019.loops.Loop;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.reporters.ReflectingLogDataGenerator;
import com.team195.lib.util.CachedValue;
import com.team195.lib.util.ElapsedTimer;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.MovingAverage;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.ArrayList;

public class VisionTracker extends Subsystem {
	private static VisionTracker mInstance = new VisionTracker();
	private PeriodicIO mPeriodicIO = new PeriodicIO();
	private ReflectingLogDataGenerator<PeriodicIO> mLogDataGenerator = new ReflectingLogDataGenerator<>(PeriodicIO.class);

	private TargetMode mTargetMode = TargetMode.HATCH;
	private boolean mVisionEnabled = false;

	private NetworkTable mCurrentTargetingLimelightNT;
	private CachedValue<NetworkTable> limelightFrontNT = new CachedValue<>(100, (t) -> NetworkTableInstance.getDefault().getTable("limelight-turret"));
	private CachedValue<NetworkTable> limelightBackNT = new CachedValue<>(100, (t) -> NetworkTableInstance.getDefault().getTable("limelight-back"));

	public static VisionTracker getInstance() {
		return mInstance;
	}

	private VisionTracker() {

	}

		private final Loop mLoop = new Loop() {
		@Override
		public void onFirstStart(double timestamp) {
			synchronized (VisionTracker.this) {

			}
		}

		@Override
		public void onStart(double timestamp) {
			synchronized (VisionTracker.this) {
			}
		}

		@Override
		public void onLoop(double timestamp) {
			synchronized (VisionTracker.this) {
				switch (mTargetMode) {
					case ROCKET_BALL:
						mPeriodicIO.pipelineFront = mVisionEnabled ? 1 : 0;
						mCurrentTargetingLimelightNT = limelightFrontNT.getValue();
						break;
					case HATCH:
					case CARGO_BALL:
					default:
						mPeriodicIO.pipelineBack = mVisionEnabled ? 1 : 0;
						mCurrentTargetingLimelightNT = limelightBackNT.getValue();
						break;
				}
			}
		}

		@Override
		public void onStop(double timestamp) {
			stop();
		}

		@Override
		public String getName() {
			return "VisionTracker";
		}
	};

	@Override
	public void registerEnabledLoops(ILooper in) {
		in.register(mLoop);
	}

	@Override
	public void stop() {
		setVisionEnabled(false);
	}

	@Override
	public synchronized boolean isSystemFaulted() {
		return false;
	}

	@Override
	public boolean runDiagnostics() {
		return true;
	}

	public boolean isVisionEnabled() {
		return mVisionEnabled;
	}

	public boolean isTargetFound() {
		return mVisionEnabled && mPeriodicIO.targetValid > 0;
	}

	public boolean isTargetAreaReached() { return mPeriodicIO.targetArea >= TargetingConstants.kVisionOffThreshold; }

	public double getTargetDistance() {
		return mVisionEnabled ? mPeriodicIO.targetDistance : 0;
	}

	public double getTargetHorizAngleDev() {
		return mVisionEnabled ? mPeriodicIO.targetHorizontalDeviation : 0;
	}

	public double getTargetVertAngleDev() {
		return mVisionEnabled ? mPeriodicIO.targetVerticalDeviation : 0;
	}

	public synchronized double getSkewFactor() {
		return mPeriodicIO.calculatedSkewFactor.getAverage();
	}

	public synchronized void setVisionEnabled(boolean enabled) {
		mVisionEnabled = enabled;
	}

	public synchronized void setTargetMode(TargetMode targetMode) {
		mTargetMode = targetMode;
	}

	public double getTargetSkew() {
		return mPeriodicIO.targetSkew;
	}

	@Override
	public synchronized String generateReport() {
//		return mLogDataGenerator.generateData(mPeriodicIO);

		return  "VisionXDev:" + mPeriodicIO.targetHorizontalDeviation + ";" +
				"VisionYDev:" + mPeriodicIO.targetVerticalDeviation + ";" +
				"VisionArea:" + mPeriodicIO.targetArea + ";" +
				"VisionDistance:" + mPeriodicIO.targetDistance + ";" +
				"VisionSkew:" + mPeriodicIO.calculatedSkewFactor + ";" +
				"IsVisionSystemFaulted:" + isSystemFaulted() + ";";
	}

	@Override
	public synchronized void readPeriodicInputs() {
		try {
			if (mVisionEnabled) {
				mPeriodicIO.targetValid = mCurrentTargetingLimelightNT.getEntry("tv").getDouble(0);
				mPeriodicIO.targetHorizontalDeviation = mCurrentTargetingLimelightNT.getEntry("tx").getDouble(0);
				mPeriodicIO.targetVerticalDeviation = mCurrentTargetingLimelightNT.getEntry("ty").getDouble(0);
				mPeriodicIO.targetArea = mCurrentTargetingLimelightNT.getEntry("ta").getDouble(0);
				mPeriodicIO.targetSkew = mCurrentTargetingLimelightNT.getEntry("ts").getDouble(0);
				mPeriodicIO.targetLatency = mCurrentTargetingLimelightNT.getEntry("tl").getDouble(0);
				mPeriodicIO.targetShortSide = mCurrentTargetingLimelightNT.getEntry("tshort").getDouble(0);
				mPeriodicIO.targetLongSide = mCurrentTargetingLimelightNT.getEntry("tlong").getDouble(0);
				mPeriodicIO.targetHorizontalSide = mCurrentTargetingLimelightNT.getEntry("thor").getDouble(0);
				mPeriodicIO.targetVerticalSide = mCurrentTargetingLimelightNT.getEntry("tvert").getDouble(0);
				mPeriodicIO.getPipelineValue = mCurrentTargetingLimelightNT.getEntry("getpipe").getDouble(0);
				mPeriodicIO.cameraTranslationRotation = mCurrentTargetingLimelightNT.getEntry("camtran").getDouble(0);

				try {
					double xArr[] = mCurrentTargetingLimelightNT.getEntry("tcornx").getDoubleArray(new double[]{0});
					double yArr[] = mCurrentTargetingLimelightNT.getEntry("tcorny").getDoubleArray(new double[]{0});

					if (xArr.length == yArr.length && xArr.length > 4) {
						mPeriodicIO.pointArray.clear();
						for (int i = 0; i < xArr.length; i++) {
							mPeriodicIO.pointArray.add(new Translation2d(xArr[i], yArr[i]));
						}

						Translation2d upperLeftPoint = mPeriodicIO.pointArray.get(0);
						Translation2d lowerLeftPoint = mPeriodicIO.pointArray.get(1);
						Translation2d lowerRightPoint = mPeriodicIO.pointArray.get(yArr.length - 2);
						Translation2d upperRightPoint = mPeriodicIO.pointArray.get(yArr.length - 1);

						double upperLineSlope = Math.abs((upperRightPoint.y() - upperLeftPoint.y()) / (upperRightPoint.x() - upperLeftPoint.x()));
						double lowerLineSlope = (lowerRightPoint.y() - lowerLeftPoint.y()) / (lowerRightPoint.x() - lowerLeftPoint.x());
						mPeriodicIO.calculatedSkewFactor.addNumber(Math.toDegrees(Math.atan((upperLineSlope + Math.abs(lowerLineSlope)) / 2.0)) * Math.signum(lowerLineSlope));
					} else {
						mPeriodicIO.calculatedSkewFactor.clear();
					}
				} catch (Exception ex) {
					ConsoleReporter.report(ex);
				}

//				mPeriodicIO.targetDistance = mTargetMode == TargetMode.ROCKET_BALL ?
//						(TargetingConstants.kRocketBallTargetHeight - TargetingConstants.kLimelightFrontMountedHeightToFloor) /
//								Math.atan(TargetingConstants.kLimelightFrontMountedAngleWrtFloor + mPeriodicIO.targetVerticalDeviation) :
//						(TargetingConstants.kHatchTargetHeight - TargetingConstants.kLimelightBackMountedHeightToFloor) /
//								Math.atan(TargetingConstants.kLimelightBackMountedAngleWrtFloor + mPeriodicIO.targetVerticalDeviation);
			}
			else {
				mPeriodicIO.targetValid = 0;
				mPeriodicIO.targetHorizontalDeviation = 0;
				mPeriodicIO.targetVerticalDeviation = 0;
				mPeriodicIO.targetArea = 0;
				mPeriodicIO.targetSkew = 0;
				mPeriodicIO.targetLatency = 0;
				mPeriodicIO.targetShortSide = 0;
				mPeriodicIO.targetLongSide = 0;
				mPeriodicIO.targetHorizontalSide = 0;
				mPeriodicIO.targetVerticalSide = 0;
				mPeriodicIO.getPipelineValue = 0;
				mPeriodicIO.cameraTranslationRotation = 0;
				mPeriodicIO.targetDistance = 0;
				mPeriodicIO.calculatedSkewFactor.clear();
			}
		}
		catch (Exception ex) {
			ConsoleReporter.report(ex);
		}
	}

	@Override
	public synchronized void writePeriodicOutputs() {
		try {
			limelightFrontNT.getValue().getEntry("pipeline").setNumber(mPeriodicIO.pipelineFront);
			limelightBackNT.getValue().getEntry("pipeline").setNumber(mPeriodicIO.pipelineBack);
		}
		catch (Exception ex) {
			ConsoleReporter.report(ex);
		}

//		NetworkTableEntry ledMode = mCurrentTargetingLimelightNT.getValue().getEntry("ledMode");
//		NetworkTableEntry camMode = mCurrentTargetingLimelightNT.getValue().getEntry("camMode");
//		NetworkTableEntry stream = mCurrentTargetingLimelightNT.getValue().getEntry("stream");
//		NetworkTableEntry snapshot = mCurrentTargetingLimelightNT.getValue().getEntry("snapshot");
	}

	private static class PeriodicIO {
		//Read values
		double targetValid;
		double targetHorizontalDeviation;
		double targetVerticalDeviation;
		double targetArea;
		double targetSkew;
		double targetLatency;
		double targetShortSide;
		double targetLongSide;
		double targetHorizontalSide;
		double targetVerticalSide;
		double targetDistance;
		double getPipelineValue;
		double cameraTranslationRotation;
		MovingAverage calculatedSkewFactor = new MovingAverage(10);

		ArrayList<Translation2d> pointArray = new ArrayList<>();

		//Written values
		int pipelineFront;
		int pipelineBack;
	}

	public enum TargetMode {
		HATCH,
		CARGO_BALL,
		ROCKET_BALL;
	}
}
