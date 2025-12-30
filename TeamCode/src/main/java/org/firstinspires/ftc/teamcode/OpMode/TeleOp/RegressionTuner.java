package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.Outtake.Launcher;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Turret;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

@Config
@TeleOp(name = "RegressionTuner", group = "Pedro Pathing")
public class RegressionTuner extends LinearOpMode {

    public static double dashboardTargetVelocity = 1500.0;
    public static double dashboardHoodAngle = 0.5;

    public static boolean turretTrackingEnabled = true;

    private Robot robot;

    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;
    private boolean prevA = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        robot.outtake.launcher.autoAimOn(false);

        waitForStart();

        while (opModeIsActive()) {
            robot.outtake.turret.turretState = turretTrackingEnabled
                    ? Turret.TurretState.TRACKING
                    : Turret.TurretState.FIXED_ANGLE;
            robot.outtake.launcher.autoAimOn(turretTrackingEnabled);

            robot.outtake.launcher.target = dashboardTargetVelocity;
            Launcher.target_tilt = dashboardHoodAngle;

            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;
            boolean left = gamepad1.dpad_left;
            boolean right = gamepad1.dpad_right;
            boolean a = gamepad1.a;

            if (up && !prevDpadUp) {
                dashboardTargetVelocity += 25.0;
            }
            if (down && !prevDpadDown) {
                dashboardTargetVelocity = Math.max(0, dashboardTargetVelocity - 25.0);
            }

            if (right && !prevDpadRight) {
                dashboardHoodAngle = Range.clip(dashboardHoodAngle + 0.01, 0.0, 1.0);
            }
            if (left && !prevDpadLeft) {
                dashboardHoodAngle = Range.clip(dashboardHoodAngle - 0.01, 0.0, 1.0);
            }

            if (a && !prevA) {
                if (robot.outtake.isLaunching()) {
                    robot.outtake.setOuttakeState(Outtake.OuttakeState.STOP);
                } else {
                    robot.outtake.start_feed_rapid(dashboardTargetVelocity, dashboardHoodAngle);
                }
            }

            robot.update();

            double distanceIn = robot.sensors.getDistanceToTarget(robot.sensors.getTargetX(), robot.sensors.getTargetY());
            telemetry.addLine("--- Regression Tuner ---");
            telemetry.addData("Distance to Target (in)", String.format("%.2f", distanceIn));
            telemetry.addData("Current Velocity (tps)", String.format("%.1f", robot.outtake.launcher.currentVel));
            telemetry.addData("Target Velocity (tps)", String.format("%.1f", dashboardTargetVelocity));
            telemetry.addData("Hood Angle", String.format("%.3f", dashboardHoodAngle));
            telemetry.addData("Shooting", robot.outtake.isLaunching());
            telemetry.addData("Turret Tracking", turretTrackingEnabled);
            telemetry.update();

            prevDpadUp = up;
            prevDpadDown = down;
            prevDpadLeft = left;
            prevDpadRight = right;
            prevA = a;
        }
    }
}
