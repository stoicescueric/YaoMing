package org.firstinspires.ftc.teamcode.Hardware.Outtake;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.Module;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.Shooting.ShootingSystem;
import org.firstinspires.ftc.teamcode.Hardware.Shooting.ShotSolution;
import org.firstinspires.ftc.teamcode.Hardware.Sensors;
import org.firstinspires.ftc.teamcode.Util.Caching.CachingServo;
import org.firstinspires.ftc.teamcode.Util.Globals.Alliance;
import org.firstinspires.ftc.teamcode.Util.Info;


@Config
public class Turret implements Module {
    Servo servoLeft;
    Servo servoRight;

    Sensors sensors;

    public static boolean backlashYok = false;
    public static double offsetAngle = 1;



    public enum TurretState {
        OFF,
        FIXED_ANGLE,
        FIXED_POS,
        TRACKING
    }

    public double fixed_pos = 0.5;
    public static double centerPose = 0.5;
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
    public static double cachingFix = 0.0005;
    public static double offset = 0.004;
    /**
     * Mounting rotation between the bearing to the target and the turret's own zero, radians.
     * PI because the shooter fires out the back, which is the same term the legacy path adds inside
     * its atan2. Kept here so both paths share one convention instead of hard-coding it twice.
     */
    public static double AZIMUTH_MOUNT_OFFSET = Math.PI;
    /** Seconds of servo slew the rate feedforward commands ahead by. Measure, do not guess. */
    public static double TURRET_SLEW_LEAD_S = 0.0;
    /** Last relative azimuth commanded, radians. Read by the shot gate. */
    public double commandedAzimuth = 0;
    /**
     * First-order model of where the servo actually is, radians. These servos have no encoder, so
     * the only way to know how far behind the command they are is to model them. Feeds the gate.
     */
    public double estimatedAzimuth = 0;
    /**
     * Servo response time constant, seconds, for the model of where the servo actually is.
     *
     * <p>An Axon Max MK2 at 6V covers 60 degrees in 0.115 s, and tracking errors are far smaller
     * than that, so the servo spends its time rate-limited rather than settling: a 10 degree error
     * closes in about 0.02 s. 0.05 allows for load and reversal on top of that. Still a derivation
     * from a datasheet, not a measurement.
     */
    public static double SERVO_TAU = 0.05;
    private final ElapsedTime servoTimer = new ElapsedTime();

    /** Commanded minus modelled-actual azimuth, radians. Zero when the servo has caught up. */
    public double azimuthError() { return commandedAzimuth - estimatedAzimuth; }
    public double lastPos = 0;
    public double launchingMaxError = 0.01;
    public boolean forceUpdate = false;
    /**
     * The turret servos are flashed in continuous rotation mode, so any setPosition spins them.
     * The turret is zip-tied in place; while this is true no command is ever sent, which leaves the
     * PWM on those ports disabled. azimuthError() stays 0, so the shot gate is not held shut.
     */
    public static boolean DISABLED = false;
    @Override
    public void update() {
        if (DISABLED) return;
        switch (turretState){
            case OFF:
                servoLeft.setPosition(0);
                servoRight.setPosition(0);
                break;
            case FIXED_ANGLE:

                servoLeft.setPosition(centerPose);
                servoRight.setPosition(centerPose );


                break;
            case FIXED_POS:
                servoLeft.setPosition(fixed_pos);
                servoRight.setPosition(fixed_pos);
                break;
            case TRACKING:
                double relativeAngle;
                double slewLead = 0;

                ShootingSystem shooting = robot.shooting;
                if (shooting != null && shooting.solverActive()) {
                    ShotSolution shot = shooting.getSolution();
                    relativeAngle = wrapPi(shot.azimuthRelative + AZIMUTH_MOUNT_OFFSET);
                    // Position servos, so a known tracking rate can only be fed forward by
                    // commanding ahead of where we want to be. The rate is analytic out of the solve
                    // (v_lateral/d - omega), which beats differentiating the commanded angle the way
                    // updateTurretPrediction had to.
                    slewLead = shot.requiredTurretRate * TURRET_SLEW_LEAD_S;
                } else {
                    double robotHeading = sensors.getHeading(); // radians
                    double directGlobalAngle = sensors.getShooterAngleToTarget();
                    if (useAngularComp) {
                        robotHeading = robotHeading + (sensors.getAngularVelocity() * turretLag);
                    }
                    relativeAngle = wrapPi(directGlobalAngle - robotHeading + AZIMUTH_MOUNT_OFFSET);
                }
                commandedAzimuth = relativeAngle;

                double pos = angleToTurretPosition(relativeAngle + slewLead - Math.toRadians(offsetAngle));
//                Log.w("Turret info: ","robot heading " + robotHeading + " directAngle " + directGlobalAngle + " relative Angle " + relativeAngle);
//                Log.w("Turret info: " ,"target X Y " +  backboardX + " " + backboardY + " turret pos " + pos);

                servoLeft.setPosition(pos + offset);
                servoRight.setPosition(pos - offset);
                lastPos = pos;

                double sdt = servoTimer.seconds();
                servoTimer.reset();
                if (sdt <= 0 || sdt > 0.25) sdt = 0.02;
                double target = relativeAngle + slewLead;
                estimatedAzimuth += (target - estimatedAzimuth) * (sdt / (Math.max(SERVO_TAU, 1e-3) + sdt));
                break;
        }
    }

    public void setPosFixed(double pos){
         fixed_pos = pos;
         turretState = TurretState.FIXED_POS;
    }


    public void addRemoveIncrementOffset(double increment,double sign) {
        offsetAngle = offsetAngle + (sign * increment);
    }
    public void resetOffset() {
        offsetAngle = 0;
    }

    /** Wraps to [-pi, pi]. Same job the old nested atan2(sin, cos) was doing. */
    private static double wrapPi(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    private double angleToTurretPosition(double angle) {
        double slope = (OuttakePositions.MAX_TURRET_POSITION - OuttakePositions.MIN_TURRET_POSITION) / Math.toRadians(180);

        double position = centerPose - (slope * angle);

//        double position = Range.scale(angle,
//                OuttakePositions.MIN_TURRET_ANGLE,
//                OuttakePositions.MAX_TURRET_ANGLE,
//                OuttakePositions.MIN_TURRET_POSITION,
//                OuttakePositions.MAX_TURRET_POSITION);
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
