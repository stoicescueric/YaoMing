package org.firstinspires.ftc.teamcode.OpMode.robotNou;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;

import org.firstinspires.ftc.teamcode.Util.Controllers.PDFL;
import org.firstinspires.ftc.teamcode.Util.Controllers.PIDF;

@Config
@TeleOp
public class TurretEncoder extends LinearOpMode {

    AnalogInput input;
    CRServo servoL;
    CRServo servoR;
    String name = "turretEncoder";
    public static double targetPos = 0;
    public static double kP = 0;
    public static double kS = 0;
    public static double kD = 0;
    public static double kF = 0;
    public double currentPos = 0;
    public static double sign1 = 1;
    public static double sign2 = 1;
    PDFL PID;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        input = hardwareMap.get(AnalogInput.class,name);
        servoL = hardwareMap.get(CRServoImplEx.class,"turretL");
        servoR = hardwareMap.get(CRServoImplEx.class,"turretR");


        PID = new PDFL(kP,kD,kF,kS);
        waitForStart();
        while (opModeIsActive()) {
            currentPos = input.getVoltage() / 3.3;
            PID.updateConstants(kP,kD,kF,kS);

            double power = PID.run(targetPos - currentPos);

            servoL.setPower(power * sign1);
            servoR.setPower(power * sign2);
            telemetry.addData("turret Pos",currentPos);
            telemetry.addData("turret target",targetPos);
            telemetry.addData("power",power);
            telemetry.update();
        }
    }
}
