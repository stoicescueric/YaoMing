package org.firstinspires.ftc.teamcode.OpMode.robotNou;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp(name = "TurretTest testNou", group = "Pedro Pathing")
public class TurretTest extends LinearOpMode {

    public static double servoLeft = 0.5;
    public static double servoRight = 0.5;
    public static boolean isSecondReversed = false;
    Servo shooter1,shooter2;

    private boolean joystickMode = false;
    private boolean previousA = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        shooter1 = hardwareMap.get(Servo.class,"servo1");
        shooter2 = hardwareMap.get(Servo.class,"servo2");


        waitForStart();
        while(opModeIsActive()){
            boolean currentA = gamepad1.a;
            if (currentA && !previousA) {
                joystickMode = !joystickMode;
            }
            previousA = currentA;

            if (joystickMode) {
                double joystickValue = (gamepad1.left_stick_x + 1) / 2.0;
                shooter1.setPosition(joystickValue);
                shooter2.setPosition(joystickValue);
            } else {
                shooter1.setPosition(servoLeft);
                shooter2.setPosition(servoRight);
            }

        }

    }
}
