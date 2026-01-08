package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Hardware.Intake.IntakeTransfer;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.OuttakePositions;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Util.Globals.Alliance;
import org.firstinspires.ftc.teamcode.Util.Info;
import org.firstinspires.ftc.teamcode.Util.Wrapper.GamePadController;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Launcher;


@Config

public class TeleOP extends LinearOpMode
{
    public static boolean turretTracking = true;
    public static boolean robotCentric = false;
    public static boolean flipFieldFrame = false;
    public boolean isReadyingFlywheel = false;
    public static double driverFrameOffsetDeg = -90;
    public static boolean switchToRedTeam = false;
    GamePadController gg;
    Robot robot;
    public static double pidTargetClosezone = 1600;
    public static double pidTargetFarzone  = 2100;
    public static double hoodGain = -0.02;
    Pose startPose;
    Pose startPoseRed = new Pose(13.5, 45, Math.PI/2);
    Pose startPoseBlue = new Pose(startPoseRed.getX(),startPoseRed.getY() *-1 , - startPoseRed.getHeading());
    Pose resetPoseRed = new Pose(63.92, -59.60, -Math.PI/2);
    Pose resetPoseBlue = new Pose(resetPoseRed.getX(),resetPoseRed.getY() *-1 , - resetPoseRed.getHeading());

    @Override
    public void runOpMode() throws InterruptedException {
         robot = new Robot(this);
         //60.48 58.22

        if (Info.alliance == Alliance.RED) {
            startPose = startPoseRed;
        }else {
            startPose = startPoseBlue;
        }
         gg = new GamePadController(gamepad1);

         robot.drive.setStartingPose(startPose);

         waitForStart();
         robot.drive.startTeleopDrive();
         robot.outtake.launcher.autoAimOn(true);

         while(opModeIsActive())
         {
              gg.update();
              updateDrive();

              robot.outtake.turret.turretState = turretTracking
                     ? org.firstinspires.ftc.teamcode.Hardware.Outtake.Turret.TurretState.TRACKING
                     : org.firstinspires.ftc.teamcode.Hardware.Outtake.Turret.TurretState.FIXED_ANGLE;

              outtakeUpdate();
              intakeUpdate();
              readyFlywheelAfterStall();
             //posibil sa trb sa mut robot.update inainte de stall check chestie
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
            if(Info.alliance == Alliance.RED) {
                robot.drive.setPose(resetPoseRed);
            }else {
                robot.drive.setPose(resetPoseBlue);
            }

        }
    }

    public void intakeUpdate() {
        if(gg.rightBumper()) {
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

    public void readyFlywheelAfterStall(){
        if (IntakeTransfer.useStall && robot.intakeTransfer.stallTriggeredThisLoop) {
            isReadyingFlywheel = true;
            robot.outtake.outtakeState = Outtake.OuttakeState.READY_FLYWHEEL;
        }
    }

    // formula: -0.006248 * distance + 0.001034 * ticksPerSecond + -0.682723

    public void outtakeUpdate() {
        if (gg.yOnce()) {
            Launcher.auto_aim = !Launcher.auto_aim;
            turretTracking = !turretTracking;
        }

        Outtake.OuttakeState state = robot.outtake.outtakeState;
        double x = robot.sensors.getX();
        double y = robot.sensors.getY();
        boolean inZone = robot.sensors.isInTargetZone(x, y);
        boolean isStill = robot.sensors.isRobotStill();

        boolean isLongShot = robot.sensors.shootingLong();

        if (gg.aOnce()) {


            if (state == Outtake.OuttakeState.IDLE) {
                if (inZone && isStill) {
                    // Idle and in zone: shoot immediately
                    if (isLongShot) {
                        robot.outtake.start_feed_precise(OuttakePositions.farLaunchVelocity, OuttakePositions.farLaunchTilt);
                    } else {
                        robot.outtake.start_feed_rapid(OuttakePositions.farLaunchVelocity, OuttakePositions.farLaunchTilt);
                    }
                } else {
                    // Idle and outside zone: spin up
                    isReadyingFlywheel = true;
                    robot.outtake.outtakeState = Outtake.OuttakeState.READY_FLYWHEEL;
                }
            } else if (state == Outtake.OuttakeState.READY_FLYWHEEL) {
                if (inZone && isStill) {
                    // Spinning up/in ready state and in zone: shoot
                    if (isLongShot) {
                        robot.outtake.start_feed_precise(OuttakePositions.farLaunchVelocity, OuttakePositions.farLaunchTilt);
                    } else {
                        robot.outtake.start_feed_rapid(OuttakePositions.farLaunchVelocity, OuttakePositions.farLaunchTilt);
                    }
                } else {
                    // Spinning but not in zone: STOP
                    isReadyingFlywheel = false;
                    robot.outtake.outtakeState = Outtake.OuttakeState.STOP;
                }
            } else {
                // Any other non-IDLE state: STOP
                isReadyingFlywheel = false;
                robot.outtake.outtakeState = Outtake.OuttakeState.STOP;
            }
        }

        if(state == Outtake.OuttakeState.READY_FLYWHEEL && inZone && isStill){
            if (isLongShot) {
                robot.outtake.start_feed_precise(OuttakePositions.farLaunchVelocity, OuttakePositions.farLaunchTilt);
            } else {
                robot.outtake.start_feed_rapid(OuttakePositions.farLaunchVelocity, OuttakePositions.farLaunchTilt);
            }
        }

        if(gg.bOnce()) {
            if(robot.outtake.outtakeState == Outtake.OuttakeState.IDLE ) {
                robot.outtake.start_feed_precise(OuttakePositions.farLaunchVelocity,OuttakePositions.farLaunchTilt);

            } else if(isReadyingFlywheel) {
                isReadyingFlywheel = false;
                robot.outtake.start_feed_precise(OuttakePositions.farLaunchVelocity, OuttakePositions.farLaunchTilt);
            }
            else{
                robot.outtake.outtakeState = Outtake.OuttakeState.STOP;
            }
        }


        if(gg.dpadUpOnce()){
            turretTracking = !turretTracking;
        }


    }
}
