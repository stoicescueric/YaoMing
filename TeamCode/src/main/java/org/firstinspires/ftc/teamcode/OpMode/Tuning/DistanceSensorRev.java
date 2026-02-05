package org.firstinspires.ftc.teamcode.OpMode.Tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp(name = "Distance Sensor Test")
public class DistanceSensorRev extends LinearOpMode {
    DistanceSensor distanceSensor;
    public static String distanceSensorName = "distanceSensor";
    @Override
    public void runOpMode() throws InterruptedException {
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, distanceSensorName);
        waitForStart();
        while (opModeIsActive()) {
            double distanceMM = distanceSensor.getDistance(DistanceUnit.MM);
            double distanceCM = distanceSensor.getDistance(DistanceUnit.CM);
            double distanceIN = distanceSensor.getDistance(DistanceUnit.INCH);

            telemetry.addData("Distance (mm)", distanceMM);
            telemetry.addData("Distance (cm)", distanceCM);
            telemetry.addData("Distance (inches)", distanceIN);
            telemetry.update();
        }
    }
}
