package org.firstinspires.ftc.teamcode.OpMode.robotNou;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Util.Wrapper.GamePadController;

@Config
@TeleOp(name = "Test Turreta")
public class TestTurreta extends LinearOpMode {
    public static double leftTurretPos = 0.5;
    public static double rightTurretPos = 0.5;
    AnalogInput turretEncoder;

    public Servo servoLeft;
    public Servo servoRight;
    GamePadController gg;

    @Override
    public void runOpMode() throws InterruptedException {
        gg = new GamePadController(gamepad1);
        servoLeft = hardwareMap.get(Servo.class, "turretL");
        servoRight = hardwareMap.get(Servo.class, "turretR");

        waitForStart();
        while(opModeIsActive()) {
            gg.update();
            if(gg.aOnce()) {
                servoLeft.setPosition(leftTurretPos);
                servoRight.setPosition(rightTurretPos);
            }

           telemetry.update();
        }
    }
}
