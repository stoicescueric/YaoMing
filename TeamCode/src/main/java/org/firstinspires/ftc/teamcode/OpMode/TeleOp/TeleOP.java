package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Drive.p2p.follower.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Intake.IntakeTransfer;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.OuttakePositions;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Util.Wrapper.GamePadController;

@Config
@TeleOp(name = "TeleOP")
public class TeleOP extends LinearOpMode
{
    GamePadController gg;
    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
         robot = new Robot(this);

         gg = new GamePadController(gamepad1);

         waitForStart();
         while(opModeIsActive())
         {
             gg.update();
             updateDrive();
             outtakeUpdate();
             intakeUpdate();
             robot.update();
         }
    }
    public void updateDrive() {
        robot.drive.setDriveMode(Drivetrain.DriveMode.ROBOT_CENTRIC);
        double forward = -gg.left_stick_y;
        double strafe = -gg.left_stick_x;
        double rotate = -gg.right_stick_x;
        robot.drive.drive(forward, strafe, rotate);
    }

    public void intakeUpdate() {
        if(gg.rightBumperOnce()) {
            if(robot.intakeTransfer.intakeState == IntakeTransfer.IntakeState.OFF) {
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.INTAKE);
            } else {
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF);
            }
        }
    }

    public void outtakeUpdate() {
        if(gg.dpadUpOnce()) {
            robot.outtake.launcher.increaseDecreaseTarget(1);
        }
        if(gg.dpadDownOnce()) {
            robot.outtake.launcher.increaseDecreaseTarget(-1);
        }

        if(gg.dpadLeftOnce()) {
            robot.outtake.turret.fixedPos+=0.05;
        }

        if(gg.dpadRightOnce()) {
            robot.outtake.turret.fixedPos-=0.05;
        }

        if(gg.aOnce()) {
            if(robot.outtake.outtakeState == Outtake.OuttakeState.IDLE ) {
                robot.outtake.start_feed(OuttakePositions.farLaunchVelocity,OuttakePositions.farLaunchTilt);

            } else {
                robot.outtake.outtakeState = Outtake.OuttakeState.STOP;
            }
        }
        if(gg.bOnce()) {
            if(robot.outtake.outtakeState == Outtake.OuttakeState.IDLE ) {
                robot.outtake.start_feed(OuttakePositions.closeLaunchVelocity,OuttakePositions.closeLaunchTilt);
            } else {
                robot.outtake.outtakeState = Outtake.OuttakeState.STOP;
            }
        }

    }
}
