package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Intake.IntakeTransfer;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.OuttakePositions;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Turret;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Util.Globals.Alliance;
import org.firstinspires.ftc.teamcode.Util.Globals.Phase;
import org.firstinspires.ftc.teamcode.Util.Info;
import org.firstinspires.ftc.teamcode.Util.Wrapper.GamePadController;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Launcher;
import org.firstinspires.ftc.teamcode.Util.Wrapper.TelemetryUtil;
import org.firstinspires.ftc.teamcode.blob.driveTrain.Blob;


@Config
public class TeleOP extends LinearOpMode
{
    public static boolean robotCentric = false;
    public static boolean flipFieldFrame = false;
    public boolean isReadyingFlywheel = false;
    public static double driverFrameOffsetDeg = -90;
    GamePadController gg;
    Robot robot;

    public static boolean isAutoAimOff = false;

    public static boolean shootWhileMoving = false;

    public static double pidTargetClosezone = 1600;
    public static double pidTargetFarzone  = 2100;

    public static boolean enableDriveSlowMode = false;
    public static boolean enableHeadingSlowMode = false;
    ElapsedTime motorTimer = new ElapsedTime();


    Pose startPose;
    Pose startPoseRed = new Pose(-3.5, 24.7, Math.PI/2);
    Pose startPoseBlue = new Pose(startPoseRed.getX(),startPoseRed.getY() *-1 , - startPoseRed.getHeading());
    Pose resetPoseRed = new Pose(58.52, -65.90, 0); //TODO
    Pose resetPoseBlue = new Pose(resetPoseRed.getX(),resetPoseRed.getY() *-1 , - resetPoseRed.getHeading());
    Pose resetCenter = new Pose(0, 0, Math.PI/2);
    private double loopTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Info.phase = Phase.TELEOP;
        Info.useBlob = false;
        robot = new Robot(this);
        //60.48 58.22

        if (Info.hasLastPose) {
            startPose = new Pose(Info.lastPoseX, Info.lastPoseY, Info.lastPoseHeading);
        } else {
            if (Info.alliance == Alliance.RED) {
                startPose = startPoseRed;
            } else {
                startPose = startPoseBlue;
            }
        }
        gg = new GamePadController(gamepad1);

        robot.blob.odo.setPose(startPose);
        robot.blob.odo.update();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        robot.outtake.launcher.autoAimOn(true);
        robot.outtake.turret.setPosFixed(0.505);
        robot.sensors.setUsePredictivePose(true);
        motorTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        while(opModeIsActive())
        {
            gg.update();
            updateDrive();


            robot.outtake.setShootingWhileMoving(shootWhileMoving);

            outtakeUpdate();
            intakeUpdate();
            telemetry.addData("intake State",robot.intakeTransfer.intakeState);
            telemetry.addData("launcer State",robot.outtake.launcher.launcherState);
            telemetry.addData("timer", motorTimer.seconds());  double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            telemetry.addData("sotm",robot.sensors.sotm);
            telemetry.addData("distance to backboard", robot.sensors.getDistanceToBackboard());


            loopTime = loop;
            telemetry.update();
            robot.update();
        }
    }
    //posibil sa trb sa mut robot.update inainte de stall check chestie




    public static double translationalSlow = 1;
    public static double rotateSlow = 0.6;
    public static double rotateNormal = 0.85;
    public static double translationalNormal = 1;
    public static double driveSlowMultiplier = 0.9;
    public static double headingSlowMultiplier = 0.9;
    public static boolean useSlowZone = false;
    public void updateDrive() {
        double forward = -gg.left_stick_y;
        double strafe = gg.left_stick_x;
        double rotate = gg.right_stick_x;


        forward *= translationalNormal;
        strafe  *= rotateNormal;


        if (enableDriveSlowMode) {
            forward *= driveSlowMultiplier;
            strafe  *= driveSlowMultiplier;
        }

        if (enableHeadingSlowMode) {
            rotate *= headingSlowMultiplier;
        }

        if (robot.sensors.isInSlowZone() && useSlowZone) {
            forward *= driveSlowMultiplier;
            strafe  *= driveSlowMultiplier;
            rotate  *= headingSlowMultiplier;
        }

        robot.blob.setMode(Blob.State.DRIVE);
        double X=strafe;
        double Y=forward;
        double rotation=rotate;

        double allianceRotation = (Info.alliance == Alliance.BLUE) ? Math.PI : 0;
        double heading = -robot.blob.odo.getHeading() + Math.PI/2 + allianceRotation;

        double x=X*Math.cos(heading)-Y*Math.sin(heading);
        double y=X* Math.sin(heading)+Y*Math.cos(heading);

        robot.blob.setTargetVector( x , y , rotation );

        if (gg.rightStickButtonOnce()) {
            if (Info.alliance == Alliance.RED) {
                robot.blob.odo.setPose(resetPoseRed);
                robot.outtake.turret.resetOffset();
            }else {
                robot.blob.odo.setPose(resetPoseBlue);
                robot.outtake.turret.resetOffset();
            }

        }
        if (gg.dpadDown()) {
            if(gg.leftTrigger() && gg.rightTrigger()){
                robot.blob.odo.setPose(resetCenter);
                robot.outtake.turret.resetOffset();
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



    public static double incrementTurret = 0.01;
    public void outtakeUpdate() {

//        if (gg.xOnce()) {
//            shootWhileMoving = !shootWhileMoving;
//        }

        Outtake.OuttakeState state = robot.outtake.outtakeState;
        double x = robot.sensors.getX();
        double y = robot.sensors.getY();
        boolean inZone = robot.sensors.isInTargetZone(x, y);
        inZone = true;

        boolean isStill = true;
        if(gg.dpadRightOnce()) {
            robot.outtake.turret.addRemoveIncrementOffset(incrementTurret,1);
        }else if(gg.dpadLeftOnce()) {
            robot.outtake.turret.addRemoveIncrementOffset(incrementTurret,-1);
        }


        if (gg.aOnce() || gg.leftBumperOnce() ) { //&& robot.intakeTransfer.intakeState != IntakeTransfer.IntakeState.INTAKE
            if(robot.intakeTransfer.isRecycle()) {
                robot.intakeTransfer.spinUpRecycle();
            }else {
                if (state == Outtake.OuttakeState.IDLE) {
                    if (inZone) {
                        // Idle and in zone: shoot immediately
                        robot.outtake.start_feed_rapid(OuttakePositions.farLaunchVelocity, OuttakePositions.farLaunchTilt);

                    }
                } else {
                    // Any other non-IDLE state: STOP
                    isReadyingFlywheel = false;
                    robot.outtake.outtakeState = Outtake.OuttakeState.STOP;
                }
            }
        }
        if(gg.startOnce()) {
            robot.sensors.toggleSOTM();
        }

        if(gg.yOnce()) {
            robot.intakeTransfer.startRecycle(false);
        }
        if(gg.xOnce()){
            robot.intakeTransfer.startRecycle(true);
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
            if(robot.outtake.turret.turretState == Turret.TurretState.TRACKING) {
                robot.outtake.turret.turretState = Turret.TurretState.FIXED_ANGLE;
            }else {
                robot.outtake.turret.turretState = Turret.TurretState.TRACKING;
            }
        }


    }
}