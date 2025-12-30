package org.firstinspires.ftc.teamcode.Hardware.Outtake;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.Module;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.Sensors;
import org.firstinspires.ftc.teamcode.Util.Caching.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.Util.Caching.CachingServo;
import org.firstinspires.ftc.teamcode.Util.Controllers.FlyWheelPID;
import org.firstinspires.ftc.teamcode.Util.HardwareUtils;
import org.firstinspires.ftc.teamcode.Util.Math.MultipleRegression;

@Config
public  class Launcher implements Module {
    CachingDcMotorEx motor1,motor2;

    CachingServo tilt;

    MultipleRegression hoodRegression = new MultipleRegression();
    InterpLUT idealVelocity = new InterpLUT();

    public enum LauncherState {
        OFF,
        SPIN_UP,
        SHOOT_STARTED,

        STEADY_VELOCITY,
        LAUNCHING,
        READY_FLYWHEEL
    }


    Sensors sensors;
    public double target = 0;
    public static boolean auto_aim = true;

    public double currentVel = 0;
    public LauncherState launcherState = LauncherState.OFF;
    public FlyWheelPID pid = new FlyWheelPID();
    Robot robot;
    public Launcher(Robot robot, Sensors sensors) {
        this.robot = robot;
        this.motor1 = new CachingDcMotorEx(robot.hw.get(DcMotorEx.class,"shooter1"),0);
        this.motor2 = new CachingDcMotorEx(robot.hw.get(DcMotorEx.class,"shooter2"),0);

        tilt = new CachingServo(robot.hw.get(Servo.class,"TurretTilt"));
        HardwareUtils.unlock(motor1);
        HardwareUtils.unlock(motor2);
        addData();
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setDirection(DcMotorEx.Direction.REVERSE);
        this.sensors = sensors;
    }

    public void addData() {

        // --- Ideal Velocity Data ---
        idealVelocity.add(36.53, 1370);
        idealVelocity.add(43.46, 1475);
        idealVelocity.add(51.22, 1480);
        idealVelocity.add(57.55, 1525);
        idealVelocity.add(59.49, 1600);
        idealVelocity.add(62.22, 1540);
        idealVelocity.add(66.33, 1555);
        idealVelocity.add(71.93, 1560);
        idealVelocity.add(75.18, 1575);
        idealVelocity.add(79.21, 1620);
        idealVelocity.add(82.71, 1635);

        idealVelocity.add(85.16, 1650);
        idealVelocity.add(89.22, 1660);
        idealVelocity.add(93.14, 1680);
        idealVelocity.add(96.28, 1745);
        idealVelocity.add(100.91, 1840);


        // --- Hood Regression Data ---

        // Distance: 36.53
        hoodRegression.add(36.53, 1320, 0.04);
        hoodRegression.add(36.53, 1345, 0.05);
        hoodRegression.add(36.53, 1370, 0.07);

        //Distance: 40.27
        hoodRegression.add(40.27, 1380, 0.13);
        hoodRegression.add(40.27, 1420, 0.11);
        hoodRegression.add(40.27, 1470, 0.1);


        // Distance: 43.46
        hoodRegression.add(43.46, 1370, 0.15);
        hoodRegression.add(43.46, 1445, 0.14);
        hoodRegression.add(43.46, 1495, 0.10);

        //Distance: 49.462
        hoodRegression.add(49.46, 1370, 0.17);
        hoodRegression.add(49.46, 1420, 0.13);
        hoodRegression.add(49.46, 1470, 0.1);

        // Distance: 51.22
        hoodRegression.add(51.22, 1380, 0.17);
        hoodRegression.add(51.22, 1420, 0.15);
        hoodRegression.add(51.22, 1480, 0.13);

        //Distance: 54.29
        hoodRegression.add(54.29, 1420, 0.15);
        hoodRegression.add(54.29, 1470, 0.13);
        hoodRegression.add(54.29, 1500, 0.11);

        // Distance: 57.55 
        hoodRegression.add(57.55, 1420, 0.18);
        hoodRegression.add(57.55, 1495, 0.20);
        hoodRegression.add(57.55, 1545, 0.16);

        // Distance: 59.49 
        hoodRegression.add(59.49, 1495, 0.14);
        hoodRegression.add(59.49, 1545, 0.18);
        hoodRegression.add(59.49, 1610, 0.17);

        // Distance: 62.22 
        hoodRegression.add(62.22, 1480, 0.14);
        hoodRegression.add(62.22, 1530, 0.18);
        hoodRegression.add(62.22, 1540, 0.19);

        // Distance: 66.33 
        hoodRegression.add(66.33, 1490, 0.14);
        hoodRegression.add(66.33, 1540, 0.16);
        hoodRegression.add(66.33, 1555, 0.17);

        // Distance: 71.93 de refacut
        //l-am refacut
        hoodRegression.add(71.93, 1510, 0.13);
        hoodRegression.add(71.93, 1545, 0.14);
        hoodRegression.add(71.93, 1570, 0.15);

        // Distance: 75.18
        hoodRegression.add(75.18, 1510, 0.13);
        hoodRegression.add(75.18, 1540, 0.12);
        hoodRegression.add(75.18, 1575, 0.16);

        // Distance: 79.21
        hoodRegression.add(79.21, 1560, 0.16);
        hoodRegression.add(79.21, 1580, 0.16);
        hoodRegression.add(79.21, 1610, 0.16);

        // Distance: 82.71
        hoodRegression.add(82.71, 1580, 0.14);
        hoodRegression.add(82.71, 1620, 0.14);
        hoodRegression.add(82.71, 1625, 0.23);

        // Distance: 85.16 
        hoodRegression.add(85.16, 1590, 0.13);
        hoodRegression.add(85.16, 1625, 0.24);
        hoodRegression.add(85.16, 1650, 0.3);

        // Distance: 89.22 
        hoodRegression.add(89.22, 1600, 0.16);
        hoodRegression.add(89.22, 1630, 0.14);
        hoodRegression.add(89.22, 1660, 0.14);

        // Distance: 93.14 
        hoodRegression.add(93.14, 1630, 0.14);
        hoodRegression.add(93.14, 1650, 0.13);
        hoodRegression.add(93.14, 1680, 0.08);

        // Distance: 96.28 
        hoodRegression.add(96.28, 1690, 0.08);
        hoodRegression.add(96.28, 1720, 0.06);
        hoodRegression.add(96.28, 1745, 0.25);

        // Distance: 100.91 
        hoodRegression.add(100.91, 1770, 0.13);
        hoodRegression.add(100.91, 1805, 0.1);
        hoodRegression.add(100.91, 1840, 0.14);


        // --- Build Tables ---
        idealVelocity.createLUT();
        hoodRegression.create();
    }
    public static double target_tilt = 0.5;
    public static double power;

    public double getPower() {
        return power;
    }

    public void changeTarget(){
        return;
    }
    @Override
    public void update() {
        currentVel = motor1.getVelocity();
        changeTarget();
        double targetDistance = sensors.getDistanceToTarget(sensors.getTargetX(), sensors.getTargetY());


        switch (launcherState){
            case OFF:
                target = 0;
                power = pid.update(target, currentVel, sensors.getVoltage());
                motor1.setPower(power);
                motor2.setPower(power);
                break;
            case SHOOT_STARTED:
                if(auto_aim){
                    try {
                        target = idealVelocity.get(targetDistance);
                        target_tilt = hoodRegression.getHoodAngle(targetDistance,target);
                    } catch (Exception e) {
                        robot.op.gamepad1.rumble(250);
                        robot.outtake.setOuttakeState(Outtake.OuttakeState.IDLE);
                        break;
                    }
                }
                launcherState = LauncherState.SPIN_UP;
                break;
            case READY_FLYWHEEL:
                try {
                    target = idealVelocity.get(targetDistance);
                } catch (Exception e) {
                    robot.op.gamepad1.rumble(250);
                    target = OuttakePositions.defaultVel;
                }
                power = pid.update(target,currentVel,sensors.getVoltage());
                motor1.setPower(power);
                motor2.setPower(power);
                break;
            case SPIN_UP:
                power = pid.update(target,currentVel,sensors.getVoltage());
                motor1.setPower(power);
                motor2.setPower(power);
                break;
            case LAUNCHING:
                if(auto_aim) {
                    try {
                        target = idealVelocity.get(targetDistance);
                        target_tilt = hoodRegression.getHoodAngle(targetDistance,currentVel);
                    } catch (Exception e) {
                        robot.op.gamepad1.rumble(250);
                        robot.outtake.setOuttakeState(Outtake.OuttakeState.IDLE);
                        break;
                    }
                }
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
        launcherState = LauncherState.SHOOT_STARTED;
    }

    public void setFlywheel(double target) {
        this.target = target;
        launcherState = LauncherState.SPIN_UP;
    }

    public void setTargetTPS(double targetTPS){
        this.target=targetTPS;

    }


    public boolean  isReady(){
        if(Math.abs(currentVel - target) < OuttakePositions.errorVelThreeshold){
            return true;
        }
        return false;
    }
    public void autoAimOn(boolean on){
        auto_aim = on;
    }


}