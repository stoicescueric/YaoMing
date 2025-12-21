package org.firstinspires.ftc.teamcode.Watchdog;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.watchdog.Watchdog;

/**
 * Simple sandbox OpMode that shows how to use Watchdog from user code.
 * It logs driver inputs every second and publishes Watchdog state to telemetry.
 */
@TeleOp(name = "Watchdog Demo", group = "Watchdog")
public class WatchdogDemoOpMode extends OpMode {
    private long lastLogTime;

    @Override
    public void init() {
        WatchdogConfig.initialize(hardwareMap.appContext);
        telemetry.addLine("Watchdog initialized. Open http://<hub-ip>:8024/ to inspect logs.");
    }

    @Override
    public void loop() {
        long now = System.currentTimeMillis();
        if (now - lastLogTime > 1000) {
            Watchdog.log(
                    "demo",
                    String.format("drive=(%.2f, %.2f)", gamepad1.left_stick_x, gamepad1.left_stick_y),
                    "inputs");
            Watchdog.log("demo", "buttons=" + buttonsPressed(), "inputs");
            lastLogTime = now;
        }

        Watchdog.pushStateToTelemetry(telemetry);
        telemetry.addData("Drive X", gamepad1.left_stick_x);
        telemetry.addData("Drive Y", gamepad1.left_stick_y);
        telemetry.update();
    }

    private String buttonsPressed() {
        StringBuilder sb = new StringBuilder();
        if (gamepad1.a) sb.append('A');
        if (gamepad1.b) sb.append('B');
        if (gamepad1.x) sb.append('X');
        if (gamepad1.y) sb.append('Y');
        if (gamepad1.left_bumper) sb.append(" LB");
        if (gamepad1.right_bumper) sb.append(" RB");
        if (gamepad1.dpad_up) sb.append(" DU");
        if (gamepad1.dpad_down) sb.append(" DD");
        if (gamepad1.dpad_left) sb.append(" DL");
        if (gamepad1.dpad_right) sb.append(" DR");
        if (sb.length() == 0) return "none";
        return sb.toString();
    }
}
