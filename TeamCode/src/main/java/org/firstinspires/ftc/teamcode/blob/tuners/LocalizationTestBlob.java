package org.firstinspires.ftc.teamcode.blob.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blob.localization.Odometry;


@TeleOp
public class LocalizationTestBlob extends LinearOpMode {

    Odometry odo;

    @Override
    public void runOpMode() throws InterruptedException {

        odo = new Odometry(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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
