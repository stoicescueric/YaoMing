package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

public interface Module {
    void update();
    default void updateTelemetry(MultipleTelemetry telemetry) {
        // Default implementation does nothing
    }
}
