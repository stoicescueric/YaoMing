package org.firstinspires.ftc.teamcode.OpMode.Tuning;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Util.Globals.Phase;
import org.firstinspires.ftc.teamcode.Util.Info;
import org.firstinspires.ftc.teamcode.blob.localization.GoBildaPinpointDriver;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
@TeleOp(name = "Pinpoint High-Rate Logger", group = "Test")
public class PinpointHighRateLogger extends LinearOpMode {
    Robot robot;
    Pose resetCenter = new Pose(0, 0, Math.PI/2);
    @Override
    public void runOpMode() throws InterruptedException {

        Info.phase = Phase.TELEOP;
        Info.useBlob = false;
        robot = new Robot(this);
        // Get hardware
        telemetry.addLine("Pinpoint ready. Press START to log.");
        telemetry.update();

        robot.drive.setStartingPose(resetCenter);

        waitForStart();
        robot.drive.startTeleopDrive();
        robot.outtake.launcher.autoAimOn(true);

        // Safe writable directory
        File folder = AppUtil.ROBOT_DATA_DIR; // guaranteed writable
        if (!folder.exists()) {
            folder.mkdirs(); // create folder if needed
        }
        File file = new File(folder, "pinpoint_log.csv");
        telemetry.addData("Logging to:", file.getAbsolutePath());
        telemetry.update();
        try (FileWriter writer = new FileWriter(file)) {
            // CSV header
            writer.write("time,x_cm,y_cm,heading_deg\n");
            long startTime = System.nanoTime();
            // Main logging loop
            while (opModeIsActive()) {
                // Update Pinpoint data
                robot.update();
                double x = robot.sensors.getX()*2.54;
                double y = robot.sensors.getY()*2.54;
                double heading = Math.toDegrees(robot.sensors.getHeading());
                double time = (System.nanoTime() - startTime) / 1e9;
                // Write CSV line
                writer.write(time + "," + x + "," + y + "," + heading + "\n");
                // Flush to ensure data is written immediately
                writer.flush();
                // Telemetry for debugging
                telemetry.addData("Time (s)", "%.2f", time);
                telemetry.addData("X (cm)", "%.2f", x);
                telemetry.addData("Y (cm)", "%.2f", y);
                telemetry.addData("Heading (deg)", "%.2f", heading);
                telemetry.update();
                // Prevent watchdog stop
                idle();
            }
        } catch (IOException e) {
            telemetry.addData("File write error", e.getMessage());
            telemetry.update();
        }
        telemetry.addLine("Logging complete.");
        telemetry.update();
    }
}