package org.firstinspires.ftc.teamcode.Hardware.Outtake;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.Module;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.Sensors;
import org.firstinspires.ftc.teamcode.Util.Caching.CachingServo;
import org.firstinspires.ftc.teamcode.Util.Globals.Alliance;
import org.firstinspires.ftc.teamcode.Util.Info;


@Config
public class Turret implements Module {
    ServoImplEx servoLeft;
    ServoImplEx servoRight;

    Sensors sensors;

    public static boolean backlashYok = false;
    public static double offset = 0;

    public static double gearRatio = 0.833;


    public enum TurretState {
        OFF,
        FIXED_ANGLE,
        TRACKING
    }
    AnalogInput input;

    public static double centerPose = 0.485;
    public TurretState turretState = TurretState.TRACKING;
    Robot robot;

    public static double lastAdjustedGlobalAngle = 0.0;

    public Turret(Robot rb, Sensors sensors){
        this.robot = rb;
        input = rb.hw.get(AnalogInput.class,"turretEncoder");
        servoLeft = rb.hw.get(ServoImplEx.class,"turretL");
        servoRight = rb.hw.get(ServoImplEx.class,"turretR");
        servoRight.setPwmRange(new PwmControl.PwmRange(500,2500));
        servoLeft.setPwmRange(new PwmControl.PwmRange(500,2500));
        this.sensors = sensors;
        resetOffset();


    }
    public static boolean useAngularComp = false;
    public static double turretLag = 0.1;
    double turretPos;
    @Override
    public void update() {
        turretPos = input.getVoltage() / 3.3;
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


                double directGlobalAngle = sensors.getShooterAngleToTarget();


                double pos = newAngleToPos(directGlobalAngle);
                Log.w("Turret info: " ,"target X Y " +  sensors.getTargetX() + " " + sensors.getTargetY() + " turret pos " + pos);
                Log.w("Turret info: ","" + turretPos);
                servoLeft.setPosition(pos );
                servoRight.setPosition(pos );
                break;
        }
    }

    public void setPosFixed(double pos){
        centerPose = pos;
    }


    public double getPos() {
        return turretPos;
    }
    public void addRemoveIncrementOffset(double increment,double sign) {
        offset = offset + (sign * increment);
    }
    public void resetOffset() {
        offset = 0;
    }

    private double newAngleToPos(double angle) {
        angle = AngleUnit.normalizeRadians(angle);
        double pos = centerPose - (gearRatio * Math.toDegrees(angle)) / 355.0;
        return Range.clip(pos,OuttakePositions.MIN_TURRET_RANGE,OuttakePositions.MAX_TURRET_RANGE);
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
