package org.firstinspires.ftc.teamcode.OpMode.robotNou;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Intake.IntakeTransfer;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

@Config
@TeleOp
public class TuneShooterPid extends LinearOpMode {
    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        waitForStart();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF_OPEN);
        while (opModeIsActive()) {
            robot.update();
            telemetry.addData("Target Velocity", robot.outtake.launcher.getTunePidTarget());
            telemetry.addData("Current Velocity", robot.outtake.launcher.currentVel);
            telemetry.addData("power",robot.outtake.launcher.getPower());
            telemetry.update();
        }
    }
}
