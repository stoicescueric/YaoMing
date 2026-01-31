package org.firstinspires.ftc.teamcode.blob.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blob.localization.Odometry;


@TeleOp
public class LocalizationTestBlob extends LinearOpMode {
    public static double startX = 0, startY = 0, headingStartRed =  Math.toRadians(0);
    Pose startPose;


    Odometry odo;

    @Override
    public void runOpMode() throws InterruptedException {
       startPose = new Pose(startX, startY, headingStartRed);

        odo = new Odometry(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        while (opModeInInit()) {
            odo.setPose(startPose);
            odo.update();
        }
        waitForStart();

        while (opModeIsActive()){

            telemetry.addData("x", odo.getX());
            telemetry.addData("y", odo.getY());
            telemetry.addData("heading", Math.toDegrees(odo.getHeading()));
            telemetry.update();
            odo.update();
        }

    }
}
