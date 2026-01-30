package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Util.Math.SlewRateLimiter;

@Disabled

@Config
@TeleOp(name = "Drive Only TeleOP", group = "Pedro Pathing")
public class DriveOnlyTeleOP extends LinearOpMode {

    public static double translationalScale = 1.0;
    public static double rotationalScale = 1.0;

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    SlewRateLimiter x,y,rx;

    public static double slewRateFactorX = 0.5;
    public static double slewRateFactorY = 0.5;
    public static double slewRateFactorRx = 0.5;
    public static boolean useSlewLimiter = true;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        backLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
        backRight = hardwareMap.get(DcMotorEx.class, "rightBack");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        x = new SlewRateLimiter(slewRateFactorX);
        y = new SlewRateLimiter(slewRateFactorY);
        rx = new SlewRateLimiter(slewRateFactorRx);
        waitForStart();

        while (opModeIsActive()) {
            updateDrive();
        }
    }

    private void updateDrive() {
        double forward = -gamepad1.left_stick_y * translationalScale;
        double strafe = gamepad1.left_stick_x * translationalScale;
        double rotate = gamepad1.right_stick_x * rotationalScale;

        telemetry.addData("forward normal",forward);
        telemetry.addData("strafe normal",strafe);
        telemetry.addData("rotate normal",rotate);
        if(useSlewLimiter) {
            forward = y.calculate(forward);
            strafe = x.calculate(strafe);
            rotate = rx.calculate(rotate);
        }

        double flPower = forward + strafe + rotate;
        double frPower = forward - strafe - rotate;
        double blPower = forward - strafe + rotate;
        double brPower = forward + strafe - rotate;

        double max = Math.max(1.0, Math.max(Math.abs(flPower), Math.max(Math.abs(frPower), Math.max(Math.abs(blPower), Math.abs(brPower)))));
        flPower /= max;
        frPower /= max;
        blPower /= max;
        brPower /= max;

        frontLeft.setPower(flPower);
        frontRight.setPower(frPower);
        backLeft.setPower(blPower);
        backRight.setPower(brPower);


        telemetry.addData("forward slew limit",forward);
        telemetry.addData("strafe slew limit",strafe);
        telemetry.addData("rotate slew limit",rotate);
        telemetry.addData("FL", flPower);
        telemetry.addData("FR", frPower);
        telemetry.addData("BL", blPower);
        telemetry.addData("BR", brPower);
        telemetry.addData("Front left amps",frontLeft.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Front right amps",frontRight.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("back left amps",backLeft.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("back right amps",backRight.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("sum amps",frontLeft.getCurrent(CurrentUnit.AMPS)
                + backLeft.getCurrent(CurrentUnit.AMPS)
                + backRight.getCurrent(CurrentUnit.AMPS)
                + frontRight.getCurrent(CurrentUnit.AMPS));
        telemetry.update();
    }
}
