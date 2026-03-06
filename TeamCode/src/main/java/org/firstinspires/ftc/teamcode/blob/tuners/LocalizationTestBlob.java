package org.firstinspires.ftc.teamcode.blob.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.blob.localization.Odometry;


@TeleOp
public class LocalizationTestBlob extends LinearOpMode {
    public static double startX = 0, startY = 0, headingStartRed =  Math.toRadians(0);
    Pose startPose;


    Odometry odo;

    // --- Mecanum drive motors (basic) ---
    private DcMotorEx leftFront = null;
    private DcMotorEx leftBack = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx rightBack = null;

    @Override
    public void runOpMode() throws InterruptedException {
        startPose = new Pose(startX, startY, headingStartRed);

        odo = new Odometry(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize mecanum motors if present in the robot configuration. Names used elsewhere in the repo.
        try {
            leftFront = hardwareMap.get(DcMotorEx.class, "fl");
            leftBack = hardwareMap.get(DcMotorEx.class, "bl");
            rightFront = hardwareMap.get(DcMotorEx.class, "fr");
            rightBack = hardwareMap.get(DcMotorEx.class, "br");

            // typical direction setup (left side reversed)
            leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
            rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
            rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

            leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        } catch (Exception ignored) {
            // If motors not configured, we will skip driving but odometry will still run.
        }

        while (opModeInInit()) {
            odo.setPose(startPose);
            odo.update();
        }
        waitForStart();

        while (opModeIsActive()){

            // Update odometry first
            odo.update();

            // Drive with gamepad1 if motors are available
            if (leftFront != null && leftBack != null && rightFront != null && rightBack != null) {
                mecanumDriveFromGamepad();
            }

            telemetry.addData("x", odo.getX());
            telemetry.addData("y", odo.getY());
            telemetry.addData("heading", Math.toDegrees(odo.getHeading()));
            telemetry.update();
        }

    }

    // Simple robot-centric mecanum drive using gamepad1 sticks
    private void mecanumDriveFromGamepad() {
        // forward/back
        double y = -gamepad1.left_stick_y; // usually forward is negative on gamepad
        // strafe
        double x = gamepad1.left_stick_x;
        // rotation
        double rx = gamepad1.right_stick_x;

        // Optional: small deadzone
        double deadzone = 0.03;
        if (Math.abs(y) < deadzone) y = 0;
        if (Math.abs(x) < deadzone) x = 0;
        if (Math.abs(rx) < deadzone) rx = 0;

        // Compensate for imperfect strafing
        x *= 1.0;

        double fl = y + x + rx;
        double bl = y - x + rx;
        double fr = y - x - rx;
        double br = y + x - rx;

        double max = Math.max(Math.max(Math.abs(fl), Math.abs(bl)), Math.max(Math.abs(fr), Math.abs(br)));
        if (max > 1.0) {
            fl /= max;
            bl /= max;
            fr /= max;
            br /= max;
        }

        leftFront.setPower(fl);
        leftBack.setPower(bl);
        rightFront.setPower(fr);
        rightBack.setPower(br);
    }
}