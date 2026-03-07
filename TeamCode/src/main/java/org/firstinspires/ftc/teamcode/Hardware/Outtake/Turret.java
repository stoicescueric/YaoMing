package org.firstinspires.ftc.teamcode.Hardware.Outtake;

import android.util.Log;

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
    Servo servoLeft;
    Servo servoRight;

    Sensors sensors;

    public static double mechRatio = 0.83;

    public static boolean backlashYok = false;
    public static double offset = 0;



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
        servoLeft = rb.hw.get(Servo.class,"turretL");
        servoRight = rb.hw.get(Servo.class,"turretR");
        this.sensors = sensors;


    }
    public static boolean useAngularComp = false;
    public static double turretLag = 0.1;
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

                double backboardX = sensors.getTargetX();
                double backboardY = sensors.getTargetY();

                double directGlobalAngle = sensors.getShooterAngleToTarget(backboardX, backboardY);

                if(useAngularComp) {
                    robotHeading = robotHeading + (sensors.getAngularVelocity() * turretLag);
                }
                double relativeAngle = Math.atan2(
                        Math.sin(directGlobalAngle - robotHeading),
                        Math.cos(directGlobalAngle - robotHeading));

                double pos = angleToTurretPosition(relativeAngle);
//                Log.w("Turret info: ","robot heading " + robotHeading + " directAngle " + directGlobalAngle + " relative Angle " + relativeAngle);
//                Log.w("Turret info: " ,"target X Y " +  backboardX + " " + backboardY + " turret pos " + pos);

                servoLeft.setPosition(pos + offset);
                servoRight.setPosition(pos + offset);
                break;
        }
    }

    public void setPosFixed(double pos){
        centerPose = pos;
    }


    private double angleToTurretPosition(double angle) {
        double position = Range.scale(angle,
                OuttakePositions.MIN_TURRET_ANGLE,
                OuttakePositions.MAX_TURRET_ANGLE,
                OuttakePositions.MIN_TURRET_POSITION,
                OuttakePositions.MAX_TURRET_POSITION);
        return Range.clip(position,
                OuttakePositions.MIN_TURRET_RANGE,
                OuttakePositions.MAX_TURRET_RANGE);
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
