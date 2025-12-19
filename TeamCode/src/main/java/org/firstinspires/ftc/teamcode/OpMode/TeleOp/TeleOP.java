package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Hardware.Intake.IntakeTransfer;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.OuttakePositions;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Util.Wrapper.GamePadController;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Launcher;


@Config
@TeleOp(name = "TeleOP")
public class TeleOP extends LinearOpMode
{
    public static boolean turretTracking = true;
    public static boolean robotCentric = false;
    public static boolean flipFieldFrame = false;
    public static double driverFrameOffsetDeg = -90;
    public static boolean switchToRedTeam = false;
    GamePadController gg;
    Robot robot;
    public static double pidTargetClosezone = 1600;
    public static double pidTargetFarzone  = 2100;
    private boolean dynamicHoodEnabled = false;
    private double lastHoodAngle = 0.0;
    public static double hoodGain = -0.02;

    @Override
    public void runOpMode() throws InterruptedException {
         robot = new Robot(this);
         Pose startPose = new Pose(60.48, 58.22, Math.PI/2);
        if (switchToRedTeam) {
            startPose = new Pose(60, -60, -Math.PI/2);
        }
         gg = new GamePadController(gamepad1);

         robot.drive.setStartingPose(startPose);

         waitForStart();
         robot.drive.startTeleopDrive();

         while(opModeIsActive())
         {
             gg.update();
             updateDrive();


             //applyLauncherTargetByX();
             //commented out pentru ca strica controll-ul de schimbat velocity (increas target/decrease target)


             robot.outtake.turret.turretState = turretTracking
                     ? org.firstinspires.ftc.teamcode.Hardware.Outtake.Turret.TurretState.TRACKING
                     : org.firstinspires.ftc.teamcode.Hardware.Outtake.Turret.TurretState.FIXED_ANGLE;

             outtakeUpdate();
             intakeUpdate();
             robot.update();
         }
    }



    private void applyLauncherTargetByX() {
        double x = robot.sensors.getX();
        if (x > 17) {
            robot.outtake.launcher.setTargetTPS(pidTargetFarzone);
        } else {
            robot.outtake.launcher.setTargetTPS(pidTargetClosezone);
        }
    }
    public static double translationalSlow = 1;
    public static double rotateSlow = 0.6;
    public static double rotateNorma = 0.8;
    public void updateDrive() {
        double forward = -gg.left_stick_y;
        double strafe = -gg.left_stick_x;
        double rotate = -gg.right_stick_x;

        double offsetRad = Math.toRadians(driverFrameOffsetDeg) + (flipFieldFrame ? Math.PI : 0.0);
        if (offsetRad != 0.0) {
            double f = forward * Math.cos(offsetRad) - strafe * Math.sin(offsetRad);
            double s = forward * Math.sin(offsetRad) + strafe * Math.cos(offsetRad);
            forward = f;
            strafe = s;
        }

        if(robot.intakeTransfer.intakeState == IntakeTransfer.IntakeState.INTAKE
                || robot.outtake.isLaunching()){
            forward*=translationalSlow;
            strafe*=translationalSlow;
            rotate*=rotateSlow;
        }else {
            rotate *= rotateNorma;
        }
        robot.drive.setTeleOpDrive(forward, strafe, rotate, robotCentric);

        if (gg.rightStickButtonOnce()) {
            Pose p = robot.drive.getPose();
            if(!switchToRedTeam){
                robot.drive.setPose(new Pose(60.48, 58.22, Math.PI/2));
            }
            else {
                robot.drive.setPose(new Pose(p.getX(), p.getY(), -Math.PI/2));
            }
            robot.drive.update();
            return;
        }

        robot.drive.update();
    }

    public void intakeUpdate() {
        if(gg.rightBumper()) {
            if(gg.rightBumperLong()){
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.REVERSE);
            }
            if(gg.rightBumperOnce()){
                switch (robot.intakeTransfer.intakeState){
                    case INTAKE:
                    case REVERSE:
                        robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF);
                        break;
                    case OFF:
                        robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.INTAKE);
                        break;
                }
            }
        }
    }

    // formula: -0.006248 * distance + 0.001034 * ticksPerSecond + -0.682723

    public void outtakeUpdate() {
        if(gg.dpadUpOnce()) {
            robot.outtake.launcher.increaseDecreaseTarget(1);
        }
        if(gg.dpadDownOnce()) {
            robot.outtake.launcher.increaseDecreaseTarget(-1);
        }

        if (gg.yOnce()) {
            Launcher.auto_aim = !Launcher.auto_aim;
            turretTracking = !turretTracking;
        }

        if(gg.aOnce() || gg.bOnce()) {
            if(robot.outtake.outtakeState == Outtake.OuttakeState.IDLE ) {

                if(gg.aOnce()) robot.outtake.start_feed_rapid(OuttakePositions.farLaunchVelocity,OuttakePositions.farLaunchTilt);
                else if(gg.bOnce()) robot.outtake.start_feed_precise(OuttakePositions.farLaunchVelocity,OuttakePositions.farLaunchTilt);

            } else {
                robot.outtake.outtakeState = Outtake.OuttakeState.STOP;
            }
        }

        if(gg.xOnce()){
            if(robot.outtake.outtakeState == Outtake.OuttakeState.READY_FLYWHEEL ) {
                robot.outtake.outtakeState = Outtake.OuttakeState.STOP;

            } else {
                robot.outtake.outtakeState = Outtake.OuttakeState.READY_FLYWHEEL;
            }
        }


    }
}
