package org.firstinspires.ftc.teamcode.Hardware.Drive.p2p.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Drive.p2p.MecanumDrive;
import org.firstinspires.ftc.teamcode.Hardware.Drive.p2p.control.Pose;
import org.firstinspires.ftc.teamcode.Hardware.Drive.p2p.control.SimpleMath;
import org.firstinspires.ftc.teamcode.Hardware.Drive.p2p.follower.Drivetrain;


@TeleOp(group = "Teleop Testers")
public class LocalizationTest extends OpMode {
    private Drivetrain drivetrain;

    private final Pose startPose = new Pose(0, 0, 0);

    @Override
    public void init() {
        drivetrain = new MecanumDrive(hardwareMap);
        drivetrain.localizer.setPose(startPose);
    }

    @Override
    public void loop() {
        double forwards = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = SimpleMath.clamp(gamepad1.left_trigger - gamepad1.right_trigger - gamepad1.right_stick_x, -1, 1);

        drivetrain.drive(
                forwards + Math.signum(forwards)*0.15,
                strafe + Math.signum(strafe)*0.15,
                rotate + Math.signum(rotate)*0.15
        );

        telemetry.addData("x (cm)", drivetrain.localizer.getPose().getX());
        telemetry.addData("y (cm)", drivetrain.localizer.getPose().getY());
        telemetry.addData("heading (deg)", Math.toDegrees(drivetrain.localizer.getPose().getHeading()));

        telemetry.update();
        drivetrain.update();
    }

}