package org.firstinspires.ftc.teamcode.OpMode.robotNou;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.OuttakePositions;

@Config
@TeleOp(name = "Angle")
public class TestAngleTureta extends LinearOpMode {
    ServoImplEx turretL,turretR;

    public static double angleWanted = 0;
    public static double centerPose = 0.485;
    public static double gearRatio = 0.833;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        turretL = hardwareMap.get(ServoImplEx.class,"turretL");
        turretR = hardwareMap.get(ServoImplEx.class,"turretR");

        turretL.setPwmRange(new PwmControl.PwmRange(500,2500));
        turretR.setPwmRange(new PwmControl.PwmRange(500,2500));
        waitForStart();
        while (opModeIsActive()){
            double pos = newAngleToPos(angleWanted);
            turretL.setPosition(pos);
            turretR.setPosition(pos);
            telemetry.update();
        }


    }
    private double newAngleToPos(double angle) {
        angle = AngleUnit.normalizeDegrees(angle);
        telemetry.addData("angle",angle);
        double pos = centerPose - (gearRatio * angle) / 355.0;
        telemetry.addData("pos",pos);
        return Range.clip(pos, OuttakePositions.MIN_TURRET_RANGE,OuttakePositions.MAX_TURRET_RANGE);
    }
}
