package org.firstinspires.ftc.teamcode.Hardware.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Hardware.Module;
import org.firstinspires.ftc.teamcode.Hardware.Sensors;
import org.firstinspires.ftc.teamcode.Util.Caching.CachingServo;

@Config
public class Turret implements Module {
    CachingServo servoLeft;
    CachingServo servoRight;

    Sensors sensors;

    public static double mechRatio = 0.9;

    // Expose target for dashboard drawing
    public double targetX = -64.1;
    public double targetY = -60.1;

    public enum TurretState {
        OFF,
        FIXED_ANGLE,
        TRACKING
    }

    public double fixedPos = 0.5;
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
                servoLeft.setPosition(fixedPos);
                servoRight.setPosition(fixedPos);
                break;
            case TRACKING:
                double robotX = sensors.getX();
                double robotY = sensors.getY();
                double robotHeading = sensors.getHeading(); // radians

                double dx = targetX - robotX;
                double dy = targetY - robotY;
                double c = Math.hypot(dx, dy);
                if (c < 1e-6) {
                    double center = 0.5;
                    servoLeft.setPosition(center);
                    servoRight.setPosition(center);
                    break;
                }

                double globalAngle = Math.atan2(dy, dx); // [-PI, PI]

                double delta = Math.atan2(Math.sin(robotHeading - globalAngle), Math.cos(robotHeading - globalAngle));

                double baseScale = 0.5 / Math.PI; // 0.25 pos -> 90deg
                double scale = baseScale * mechRatio;

                double pos = 0.5 - (delta * scale);

                if (pos < 0.05) pos = 0.05;
                if (pos > 0.95) pos = 0.95;

                servoLeft.setPosition(pos);
                servoRight.setPosition(pos);
                break;
        }
    }
}
