package org.firstinspires.ftc.teamcode.OpMode.robotNou;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Util.Wrapper.TelemetryUtil;

@Config
@TeleOp(name = "Shooter", group = "Test")
public class Shooter extends LinearOpMode {
    public static double power = 0.6;
    DcMotorEx motor1,motor2,motor3;
    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = hardwareMap.get(DcMotorEx.class,"shooter1");
        motor2 = hardwareMap.get(DcMotorEx.class,"shooter2");
        motor3 = hardwareMap.get(DcMotorEx.class, "fl");
        TelemetryUtil.setup();
        waitForStart();
        while (opModeIsActive()) {
            motor1.setPower(-power);
            motor2.setPower(power);
            TelemetryUtil.packet.put("x",motor3.getVelocity());
            TelemetryUtil.sendTelemetry();
        }
    }
}
