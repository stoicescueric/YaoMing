package org.firstinspires.ftc.teamcode.Hardware.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.Module;
import org.firstinspires.ftc.teamcode.Hardware.Sensors;
import org.firstinspires.ftc.teamcode.Util.Caching.CachingServo;

@Config
public class Turret implements Module {
    CachingServo servoLeft;
    CachingServo servoRight;

    Sensors sensors;

    public static double mechRatio = 0.83;

    // Expose target for dashboard drawing


    public enum TurretState {
        OFF,
        FIXED_ANGLE,
        TRACKING
    }

    public static double centerPose = 0.5;
    public TurretState turretState = TurretState.FIXED_ANGLE;
    public Turret(HardwareMap hw, Sensors sensors){
        servoLeft = new CachingServo(hw.get(Servo.class,"turretLeft"));
        servoRight = new CachingServo(hw.get(Servo.class,"turretRight"));
        this.sensors = sensors;
    }

    @Override
    public void update() {
        switch (turretState){
            case OFF:
                servoLeft.setPosition(0);
                servoRight.setPosition(0);
                break;
            case FIXED_ANGLE:
                servoLeft.setPosition(centerPose);
                servoRight.setPosition(centerPose);
                break;
            case TRACKING:
                double robotHeading = sensors.getHeading(); // radians

                double c = sensors.getDistanceToTarget(sensors.getTargetX(),sensors.getTargetY());
                if (c < 1e-6) {
                    double center = 0.5;
                    servoLeft.setPosition(center);
                    servoRight.setPosition(center);
                    break;
                }

                double globalAngle = sensors.getAngleToTarget(sensors.getTargetX(),sensors.getTargetY());
                double relativeAngle = Math.atan2(Math.sin(globalAngle - robotHeading), Math.cos(globalAngle - robotHeading));

                double pos = angleToTurretPosition(relativeAngle);
                servoLeft.setPosition(pos);
                servoRight.setPosition(pos);
                break;
        }
    }
    private double angleToTurretPosition(double angle) {
        double position = Range.scale(angle, OuttakePositions.MIN_TURRET_ANGLE,OuttakePositions.MAX_TURRET_ANGLE,OuttakePositions.MIN_TURRET_POSITION, OuttakePositions.MAX_TURRET_POSITION);
        return Range.clip(position, OuttakePositions.MIN_TURRET_POSITION,OuttakePositions.MAX_TURRET_POSITION);
    }
}
