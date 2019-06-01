package com.team195.frc2019.subsystems;

import com.team195.frc2019.loops.ILooper;
import com.team195.frc2019.loops.Loop;
import edu.wpi.first.wpilibj.Compressor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

public class Infrastructure extends Subsystem {

    private static Infrastructure mInstance = new Infrastructure();
//    private Superstructure mSuperstructure;
//    private Intake mIntake;
    private Compressor mCompressor;

    private boolean mIsDuringAuto = false;
    private AtomicBoolean isRunning = new AtomicBoolean(false);

    private static final List<Object> emptyList = new ArrayList<>(1);

    private Infrastructure() {
        mCompressor = new Compressor(0);
        startCompressor();
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
        if (!isRunning.get()) {
            mCompressor.start();
            isRunning.set(true);
        }
    }

    private void stopCompressor() {
        if (isRunning.get()) {
            mCompressor.stop();
            isRunning.set(false);
        }
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
    public List<Object> generateReport() {
        return emptyList;
    }
}
