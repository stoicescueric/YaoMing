package org.firstinspires.ftc.teamcode.Hardware.Outtake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Hardware.Module;
import org.firstinspires.ftc.teamcode.Hardware.Sensors;
import org.firstinspires.ftc.teamcode.Util.Caching.CachingServo;

public class Turret implements Module {
    CachingServo servoLeft;
    CachingServo servoRight;

    Sensors sensors;



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
                //TODO:implement
                break;
        }
    }
}
