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


@Config
@TeleOp(name = "TeleOP")
public class TeleOP extends LinearOpMode
{
    public static boolean turretTracking = false;
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
         Pose startPose = new Pose(0, 0, -Math.PI/2);
        if (switchToRedTeam) {
            startPose = new Pose(60, -60, Math.PI/2);
        }
         gg = new GamePadController(gamepad1);

         robot.drive.setStartingPose(startPose);

         waitForStart();
         robot.drive.startTeleopDrive();

         while(opModeIsActive())
         {
             gg.update();
             updateDrive();

             updateDynamicHoodTracking();

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

    private void updateDynamicHoodTracking() {
        if (!turretTracking) return;
        if (!dynamicHoodEnabled) return;
        if (!robot.outtake.isLaunching()) return;

        double hood = computeHoodAngle();
        // Only reapply if changed enough to matter
        if (Math.abs(hood - lastHoodAngle) > 0.01) {
            robot.outtake.start_feed_precise(OuttakePositions.farLaunchVelocity, hood);
            lastHoodAngle = hood;
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
    public static double translationalSlow = 0.8;
    public static double rotateSlow = 0.5;
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
        }
        robot.drive.setTeleOpDrive(forward, strafe, rotate, robotCentric);

        if (gg.rightStickButtonOnce()) {
            Pose p = robot.drive.getPose();
            if(!switchToRedTeam){
                robot.drive.setPose(new Pose(p.getX(), p.getY(), -Math.PI/2));
            }
            else {
                robot.drive.setPose(new Pose(p.getX(), p.getY(), Math.PI/2));
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
    private double computeHoodAngle() {
        double rx = robot.sensors.getX();
        double ry = robot.sensors.getY();
        double tx = robot.outtake.turret.targetX;
        double ty = robot.outtake.turret.targetY;
        double distance = Math.hypot(tx - rx, ty - ry);
        // use current launcher target TPS (fallback to farzone if not set)
        double tps = robot.outtake.launcher.target != 0 ? robot.outtake.launcher.target : pidTargetFarzone;
        return (-0.006248 * distance) + (0.001034 * tps) + (-0.682723) +(hoodGain);
    }

    public void outtakeUpdate() {
        if(gg.dpadUpOnce()) {
            robot.outtake.launcher.increaseDecreaseTarget(1);
        }
        if(gg.dpadDownOnce()) {
            robot.outtake.launcher.increaseDecreaseTarget(-1);
        }


        // A toggles dynamic hood tracking mode when turret tracking is enabled
        if (turretTracking && gg.aOnce()) {
            if (robot.outtake.outtakeState == Outtake.OuttakeState.IDLE) {
                dynamicHoodEnabled = true;
                lastHoodAngle = computeHoodAngle();
                robot.outtake.start_feed_precise(OuttakePositions.farLaunchVelocity, lastHoodAngle);
            } else {
                dynamicHoodEnabled = false;
                robot.outtake.outtakeState = Outtake.OuttakeState.STOP;
            }
            return;
        }

        if(!turretTracking && (gg.aOnce() || gg.bOnce())) {
            if(robot.outtake.outtakeState == Outtake.OuttakeState.IDLE ) {

                if(gg.aOnce()) robot.outtake.start_feed_rapid(OuttakePositions.farLaunchVelocity,OuttakePositions.farLaunchTilt);
                else if(gg.bOnce()) robot.outtake.start_feed_precise(OuttakePositions.farLaunchVelocity,OuttakePositions.farLaunchTilt);

            } else {
                robot.outtake.outtakeState = Outtake.OuttakeState.STOP;
            }
        }

    }
}
