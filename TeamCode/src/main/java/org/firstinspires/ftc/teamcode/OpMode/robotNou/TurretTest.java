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
@TeleOp(name = "TurretTest testNou")
public class TurretTest extends LinearOpMode {

    public static double servoLeft = 0.5;
    public static double servoRight = 0.5;
    public static boolean isSecondReversed = false;
    Servo shooter1,shooter2;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        shooter1 = hardwareMap.get(Servo.class,"servo1");
        shooter2 = hardwareMap.get(Servo.class,"servo2");


        waitForStart();
        while(opModeIsActive()){
            shooter1.setPosition(servoLeft);
            shooter2.setPosition(servoRight);


        }

    }
}
