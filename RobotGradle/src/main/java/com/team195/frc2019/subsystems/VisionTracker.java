package com.team195.frc2019.subsystems;

import com.team195.frc2019.Constants;
import com.team195.frc2019.loops.ILooper;
import com.team195.frc2019.loops.Loop;
import com.team195.lib.util.CachedValue;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionTracker extends Subsystem {
	private static VisionTracker mInstance = new VisionTracker();
	private PeriodicIO mPeriodicIO = new PeriodicIO();
	private TargetMode mTargetMode = TargetMode.HATCH;
	private boolean mVisionEnabled = false;

	private NetworkTable mCurrentTargetingLimelightNT;
	private CachedValue<NetworkTable> limelightFrontNT = new CachedValue<>(100, (t) -> NetworkTableInstance.getDefault().getTable("limelightFront"));
	private CachedValue<NetworkTable> limelightBackNT = new CachedValue<>(100, (t) -> NetworkTableInstance.getDefault().getTable("limelightBack"));


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
				mPeriodicIO.pipelineFront = mVisionEnabled ? 1 : 0;
				mPeriodicIO.pipelineBack = mVisionEnabled ? 1 : 0;

				switch (mTargetMode) {
					case ROCKET_BALL:
						mCurrentTargetingLimelightNT = limelightFrontNT.getValue();
						break;
					case HATCH:
					case CARGO_BALL:
					default:
						mCurrentTargetingLimelightNT = limelightBackNT.getValue();
						break;
				}
			}
		}

		@Override
		public void onStop(double timestamp) {
			stop();
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
	public boolean isSystemFaulted() {
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

	public double getTargetDistance() {
		return mVisionEnabled ? mPeriodicIO.targetDistance : 0;
	}

	public double getTargetHorizAngleDev() {
		return mVisionEnabled ? mPeriodicIO.targetHorizontalDeviation : 0;
	}

	public synchronized void setVisionEnabled(boolean enabled) {
		mVisionEnabled = enabled;
	}

	public synchronized void setTargetMode(TargetMode targetMode) {
		mTargetMode = targetMode;
	}

	@Override
	public String generateReport() {
		return  "VisionXDev:" + mPeriodicIO.targetHorizontalDeviation + ";" +
				"VisionYDev:" + mPeriodicIO.targetVerticalDeviation + ";" +
				"VisionArea:" + mPeriodicIO.targetArea + ";" +
				"VisionDistance:" + mPeriodicIO.targetDistance + ";";
	}

	@Override
	public synchronized void readPeriodicInputs() {
		try {
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
			mPeriodicIO.targetDistance = mTargetMode == TargetMode.ROCKET_BALL ?
					(Constants.kRocketBallTargetHeight - Constants.kLimelightFrontMountedHeightToFloor) /
							Math.atan(Constants.kLimelightFrontMountedAngleWrtFloor + mPeriodicIO.targetVerticalDeviation) :
					(Constants.kHatchTargetHeight - Constants.kLimelightBackMountedHeightToFloor) /
							Math.atan(Constants.kLimelightBackMountedAngleWrtFloor + mPeriodicIO.targetVerticalDeviation);
		}
		catch (Exception ignored) {
			;
		}
	}

	@Override
	public synchronized void writePeriodicOutputs() {
		try {
		limelightFrontNT.getValue().getEntry("pipeline").setNumber(mPeriodicIO.pipelineFront);
		limelightBackNT.getValue().getEntry("pipeline").setNumber(mPeriodicIO.pipelineBack);

		}
		catch (Exception ignored) {
			;
		}

//		NetworkTableEntry ledMode = mCurrentTargetingLimelightNT.getValue().getEntry("ledMode");
//		NetworkTableEntry camMode = mCurrentTargetingLimelightNT.getValue().getEntry("camMode");
//		NetworkTableEntry stream = mCurrentTargetingLimelightNT.getValue().getEntry("stream");
//		NetworkTableEntry snapshot = mCurrentTargetingLimelightNT.getValue().getEntry("snapshot");
	}

	public static class PeriodicIO {
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