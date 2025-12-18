package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.Outtake.Launcher;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

@TeleOp(name = "RegressionTuner", group = "Pedro Pathing")
public class RegressionTuner extends LinearOpMode {

    private Robot robot;

    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;
    private boolean prevA = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);

        waitForStart();

        while (opModeIsActive()) {
            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;
            boolean left = gamepad1.dpad_left;
            boolean right = gamepad1.dpad_right;
            boolean a = gamepad1.a;

            if (up && !prevDpadUp) {
                robot.outtake.launcher.target += 25.0;
            }
            if (down && !prevDpadDown) {
                robot.outtake.launcher.target -= 25.0;
                if (robot.outtake.launcher.target < 0) robot.outtake.launcher.target = 0;
            }

            if (right && !prevDpadRight) {
                Launcher.target_tilt = Range.clip(Launcher.target_tilt + 0.01, 0.0, 1.0);
            }
            if (left && !prevDpadLeft) {
                Launcher.target_tilt = Range.clip(Launcher.target_tilt - 0.01, 0.0, 1.0);
            }

            if (a && !prevA) {
                if (robot.outtake.isLaunching()) {
                    robot.outtake.setOuttakeState(Outtake.OuttakeState.STOP);
                } else {
                    robot.outtake.start_feed_rapid(robot.outtake.launcher.target, Launcher.target_tilt);
                }
            }

            robot.update();

            double distanceIn = robot.sensors.getDistanceToTarget(robot.sensors.getTargetX(), robot.sensors.getTargetY());
            telemetry.addLine("--- Regression Tuner ---");
            telemetry.addData("Distance to Target (in)", String.format("%.2f", distanceIn));
            telemetry.addData("Current Velocity (tps)", String.format("%.1f", robot.outtake.launcher.currentVel));
            telemetry.addData("Target Velocity (tps)", String.format("%.1f", robot.outtake.launcher.target));
            telemetry.addData("Hood Angle", String.format("%.3f", Launcher.target_tilt));
            telemetry.addData("Shooting", robot.outtake.isLaunching());
            telemetry.update();

            prevDpadUp = up;
            prevDpadDown = down;
            prevDpadLeft = left;
            prevDpadRight = right;
            prevA = a;
        }
    }
}

