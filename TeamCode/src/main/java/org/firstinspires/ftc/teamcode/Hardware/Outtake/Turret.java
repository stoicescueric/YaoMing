package org.firstinspires.ftc.teamcode.Hardware.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.Module;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.Sensors;
import org.firstinspires.ftc.teamcode.Util.Caching.CachingServo;
import org.firstinspires.ftc.teamcode.Util.Globals.Alliance;
import org.firstinspires.ftc.teamcode.Util.Info;


@Config
public class Turret implements Module {
    CachingServo servoLeft;
    CachingServo servoRight;

    Sensors sensors;

    public static double mechRatio = 0.83;

    public static boolean backlashYok = false;
    public static double offset = -0.003;

    public static double BOARD1_NX = 0.16;
    public static double BOARD1_NY = 0.0;
    public static double BOARD2_NX = 0.0;
    public static double BOARD2_NY = 0.13;

    public double BOARD1_NXrl;
    public double BOARD1_NYrl;
    public double BOARD2_NXrl;
    public double BOARD2_NYrl;

    public static double BACKBOARD_AIM_GAIN = 0.08;
    public static double MAX_BACKBOARD_AIM_OFFSET = Math.toRadians(10.0);


    public static double MOTION_LEAD_GAIN = 5.6;
    public static double MIN_LEAD_SPEED = 3.0;
    public static double ANGLE_SMOOTH_ALPHA = 0;

    public static double lastMotionCompensatedAngle = 0.0;
    private static double smoothedGlobalAngle = 0.0;

    public enum TurretState {
        OFF,
        FIXED_ANGLE,
        TRACKING
    }

    public static double centerPose = 0.505;
    public TurretState turretState = TurretState.TRACKING;
    Robot robot;

    public static double lastAdjustedGlobalAngle = 0.0;

    public Turret(Robot rb, Sensors sensors){
        this.robot = rb;
        servoLeft = new CachingServo(rb.hw.get(Servo.class,"turretL"));
        servoRight = new CachingServo(rb.hw.get(Servo.class,"turretR"));
        this.sensors = sensors;

        if (Info.alliance == Alliance.RED) {
            BOARD1_NXrl = BOARD2_NX;
            BOARD1_NYrl = -BOARD2_NY;
            BOARD2_NXrl = BOARD1_NX;
            BOARD2_NYrl = BOARD1_NY;
        } else {
            BOARD1_NXrl = BOARD1_NX;
            BOARD1_NYrl = BOARD1_NY;
            BOARD2_NXrl = BOARD2_NX;
            BOARD2_NYrl = BOARD2_NY;
        }
    }

    @Override
    public void update() {
        switch (turretState){
            case OFF:
                servoLeft.setPosition(0);
                servoRight.setPosition(0);
                break;
            case FIXED_ANGLE:
                if(backlashYok) {
                    servoLeft.setPosition(centerPose + offset);
                    servoRight.setPosition(centerPose - offset);
                }else {
                    servoLeft.setPosition(centerPose);
                    servoRight.setPosition(centerPose);
                }

                break;
            case TRACKING:
                double robotHeading = sensors.getHeading(); // radians

                double backboardX = sensors.getBackboardX();
                double backboardY = sensors.getBackboardY();

                double baseDistance = sensors.getDistanceToTarget(backboardX, backboardY);
                if (baseDistance < 1e-6) {
                    double center = 0.5;
                    servoLeft.setPosition(center);
                    servoRight.setPosition(center);
                    break;
                }

                double directGlobalAngle = sensors.getAngleToTarget(backboardX, backboardY);
                double optimizedGlobalAngle = computeBackboardOptimizedAngle(
                        sensors.getX(),
                        sensors.getY(),
                        backboardX,
                        backboardY,
                        directGlobalAngle
                );

                lastAdjustedGlobalAngle = optimizedGlobalAngle;

                double finalGlobalAngle = optimizedGlobalAngle;

                if (robot.outtake != null && robot.outtake.isShootingWhileMoving() && MOTION_LEAD_GAIN != 0.0) {
                    double rx = sensors.getX();
                    double ry = sensors.getY();

                    double vx = sensors.getVelX();
                    double vy = sensors.getVelY();

                    double robotSpeed = Math.hypot(vx, vy);
                    if (robotSpeed > MIN_LEAD_SPEED && robot.outtake.launcher != null) {
                        double projSpeed = robot.outtake.launcher.getProjectileSpeedEstimate();
                        if (projSpeed > 1e-3) {
                            double shotTime = baseDistance / projSpeed;

                            double moveGoalX = backboardX;
                            double moveGoalY = backboardY;

                            for (int i = 0; i < 5; i++) {
                                double virtualGoalX = backboardX
                                        - shotTime * (vx * MOTION_LEAD_GAIN);
                                double virtualGoalY = backboardY
                                        - shotTime * (vy * MOTION_LEAD_GAIN);

                                double dx = virtualGoalX - rx;
                                double dy = virtualGoalY - ry;
                                double distToVirtual = Math.hypot(dx, dy);

                                double newShotTime = distToVirtual / projSpeed;

                                if (Math.abs(newShotTime - shotTime) <= 0.01) {
                                    moveGoalX = virtualGoalX;
                                    moveGoalY = virtualGoalY;
                                    shotTime = newShotTime;
                                    break;
                                }

                                shotTime = newShotTime;

                                if (i == 4) {
                                    moveGoalX = virtualGoalX;
                                    moveGoalY = virtualGoalY;
                                }
                            }

                            double dxLead = moveGoalX - rx;
                            double dyLead = moveGoalY - ry;
                            if (Math.hypot(dxLead, dyLead) > 1e-6) {
                                finalGlobalAngle = Math.atan2(dyLead, dxLead);
                            }
                        }
                    }
                }

                if (ANGLE_SMOOTH_ALPHA <= 0.0) {
                    smoothedGlobalAngle = finalGlobalAngle;
                } else {
                    double d = Math.atan2(
                            Math.sin(finalGlobalAngle - smoothedGlobalAngle),
                            Math.cos(finalGlobalAngle - smoothedGlobalAngle));
                    smoothedGlobalAngle = smoothedGlobalAngle + ANGLE_SMOOTH_ALPHA * d;
                }

                lastMotionCompensatedAngle = smoothedGlobalAngle;

                double relativeAngle = Math.atan2(
                        Math.sin(smoothedGlobalAngle - robotHeading),
                        Math.cos(smoothedGlobalAngle - robotHeading));

                double pos = angleToTurretPosition(relativeAngle);
                if(backlashYok) {
                    servoLeft.setPosition(pos + offset);
                    servoRight.setPosition(pos - offset);
                }else {
                    servoLeft.setPosition(pos);
                    servoRight.setPosition(pos);
                }
                break;
        }
    }

    public void setPosFixed(double pos){
        centerPose = pos;
    }

    private double computeBackboardOptimizedAngle(double robotX, double robotY,
                                                  double rimX, double rimY,
                                                  double directAngle) {
        if (BACKBOARD_AIM_GAIN == 0.0) {
            return directAngle;
        }

        double dx = rimX - robotX;
        double dy = rimY - robotY;
        double mag = Math.hypot(dx, dy);
        if (mag < 1e-6) return directAngle;
        double vx = dx / mag;
        double vy = dy / mag;

        // normalize alliance-adjusted board normals
        double b1mag = Math.hypot(BOARD1_NXrl, BOARD1_NYrl);
        double n1x = b1mag > 1e-6 ? BOARD1_NXrl / b1mag : 0.0;
        double n1y = b1mag > 1e-6 ? BOARD1_NYrl / b1mag : 0.0;
        double b2mag = Math.hypot(BOARD2_NXrl, BOARD2_NYrl);
        double n2x = b2mag > 1e-6 ? BOARD2_NXrl / b2mag : 0.0;
        double n2y = b2mag > 1e-6 ? BOARD2_NYrl / b2mag : 0.0;

        double cosInc1 = vx * n1x + vy * n1y;
        double cosInc2 = vx * n2x + vy * n2y;

        double incidenceDiff = cosInc1 - cosInc2;

        double offsetRad = BACKBOARD_AIM_GAIN * incidenceDiff;
        offsetRad = Range.clip(offsetRad, -MAX_BACKBOARD_AIM_OFFSET, MAX_BACKBOARD_AIM_OFFSET);

        return directAngle + offsetRad;
    }

    private double angleToTurretPosition(double angle) {
        double position = Range.scale(angle,
                OuttakePositions.MIN_TURRET_ANGLE,
                OuttakePositions.MAX_TURRET_ANGLE,
                OuttakePositions.MIN_TURRET_POSITION,
                OuttakePositions.MAX_TURRET_POSITION);
        return Range.clip(position,
                OuttakePositions.MIN_TURRET_POSITION,
                OuttakePositions.MAX_TURRET_POSITION);
    }
// PSA pentru prostul care schimba astea in void. Nigga se strica daca il faci void asa ca ramane double si o sa dea return 0. Yee ass NIIIIGGGGAAAAA
    public double backlashEvet(){
        backlashYok = false;
        return 0;
    }
    //functiile sunt folosite in launcher line 285
    public double backlashYok(){
        backlashYok = true;
        return 0;
    }
}
