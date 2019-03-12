package com.team195.frc2019.subsystems;

import com.team195.frc2019.loops.ILooper;
import com.team195.frc2019.loops.Loop;
import edu.wpi.first.wpilibj.Compressor;

public class Infrastructure extends Subsystem {

    private static Infrastructure mInstance = new Infrastructure();
//    private Superstructure mSuperstructure;
//    private Intake mIntake;
    private Compressor mCompressor;

    private boolean mIsDuringAuto = false;

    private Infrastructure() {
        mCompressor = new Compressor(0);
        mCompressor.start();

//        mSuperstructure = Superstructure.getInstance();
//        mIntake = Intake.getInstance();
    }

    public static Infrastructure getInstance() {
        return mInstance;
    }

    @Override
    public void stop() {
        // No-op.
    }

    @Override
    public void zeroSensors() {
        // No-op.
    }

    private void startCompressor() {
        mCompressor.start();
    }

    private void stopCompressor() {
        mCompressor.stop();
    }

    public synchronized void setIsDuringAuto(boolean isDuringAuto) {
        mIsDuringAuto = isDuringAuto;
//        if (isDuringAuto) stopCompressor();
    }

    public synchronized boolean isDuringAuto() {
        return mIsDuringAuto;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onFirstStart(double timestamp) {

            }

            @Override
            public void onStart(double timestamp) {

            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Infrastructure.this) {
//                    boolean elevatorMoving = mSuperstructure.getSuperStructureState() ==
//                            SuperstructureStateMachine.SystemState.MOVING_TO_POSITION;
//                    boolean isIntaking = mIntake.getWantedAction() == IntakeStateMachine.WantedAction.WANT_CUBE
//                            && !mIntake.hasCube();
                    if (mIsDuringAuto) {
                        //stopCompressor();
                    } else {
                        startCompressor();
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {

            }

            @Override
            public String getName() {
                return "Infrastructure";
            }
        });
    }

    @Override
    public boolean isSystemFaulted() {
        return false;
    }

    @Override
    public boolean runDiagnostics() {
        return false;
    }

    @Override
    public String generateReport() {
        return "";
    }
}
