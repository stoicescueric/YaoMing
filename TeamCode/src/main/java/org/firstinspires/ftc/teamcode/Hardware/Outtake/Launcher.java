package org.firstinspires.ftc.teamcode.Hardware.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.Module;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.Sensors;
import org.firstinspires.ftc.teamcode.Util.Caching.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.Util.Caching.CachingServo;
import org.firstinspires.ftc.teamcode.Util.Controllers.FlyWheelPID;
import org.firstinspires.ftc.teamcode.Util.HardwareUtils;

@Config
public  class Launcher implements Module {
    private HardwareMap hw;
    CachingDcMotorEx motor1,motor2;

    CachingServo tilt;
    public enum LauncherState {
        OFF,
        SPIN_UP,
        STEADY_VELOCITY
    }

    Sensors sensors;
    public double target = 0;
    public double currentVel = 0;
    public LauncherState launcherState = LauncherState.OFF;
    public FlyWheelPID pid = new FlyWheelPID();
    public Launcher(HardwareMap hw, Sensors sensors) {
        this.motor1 = new CachingDcMotorEx(hw.get(DcMotorEx.class,"shooter1"),0);
        this.motor2 = new CachingDcMotorEx(hw.get(DcMotorEx.class,"shooter2"),0);

        tilt = new CachingServo(hw.get(Servo.class,"TurretTilt"));
        HardwareUtils.unlock(motor1);
        HardwareUtils.unlock(motor2);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setDirection(DcMotorEx.Direction.REVERSE);
        this.sensors = sensors;
    }

    public static double target_tilt = 0.5;
    public static double power;

    public double getPower() {
        return power;
    }
    @Override
    public void update() {
        currentVel = motor1.getVelocity();
        switch (launcherState){
            case OFF:
                motor1.setPower(0);
                motor2.setPower(0);
                break;
            case SPIN_UP:
                power = pid.update(target,currentVel,sensors.getVoltage());
                motor1.setPower(power);
                motor2.setPower(power);
                break;

        }
        tilt.setPosition(target_tilt);
    }

    public void increaseDecreaseTarget(double delta) {
        target += (delta * 50);
    }
    public void setTarget(double target,double hood_tilt) {
        this.target = target;
        target_tilt = hood_tilt;
        launcherState = LauncherState.SPIN_UP;
    }


    public boolean  isReady(){
        if(Math.abs(currentVel - target) < OuttakePositions.errorVelThreeshold){
            return true;
        }
        return false;
    }


}