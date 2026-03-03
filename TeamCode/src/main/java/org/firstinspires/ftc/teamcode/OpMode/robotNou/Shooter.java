package org.firstinspires.ftc.teamcode.OpMode.robotNou;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name = "Shooter", group = "Test")
public class Shooter extends LinearOpMode {
    public static double power = 1;
    DcMotorEx motor1,motor2;
    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = hardwareMap.get(DcMotorEx.class,"shooter1");
        motor2 = hardwareMap.get(DcMotorEx.class,"shooter2");
        waitForStart();
        while (opModeIsActive()) {
            motor1.setPower(power);
            motor2.setPower(power);
        }
    }
}
