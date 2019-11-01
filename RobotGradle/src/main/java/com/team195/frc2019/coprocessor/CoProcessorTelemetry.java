package com.team195.frc2019.coprocessor;


public class CoProcessorTelemetry {

    private static final CoProcessorTelemetry mInstance = new CoProcessorTelemetry();

    public static CoProcessorTelemetry getInstance() {
        return mInstance;
    }
    private CoProcessorTelemetry() { }

}
