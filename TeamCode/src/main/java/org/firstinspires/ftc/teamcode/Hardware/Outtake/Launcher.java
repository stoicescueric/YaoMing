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

    MultipleRegression hoodRegressionClose = new MultipleRegression();
    MultipleRegression hoodRegressionFar = new MultipleRegression();
    InterpLUT idealVelocityClose = new InterpLUT();
    InterpLUT idealVelocityFar = new InterpLUT();

    public static double FAR_ZONE_X_THRESHOLD = 17.0;

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

        // --- Ideal Velocity Data (close zone) ---
        idealVelocityClose.add(36.53, 1370);
        idealVelocityClose.add(43.46, 1475);
        idealVelocityClose.add(51.22, 1480);
        idealVelocityClose.add(57.55, 1525);
        idealVelocityClose.add(59.49, 1600);
        idealVelocityClose.add(62.22, 1540);
        idealVelocityClose.add(66.33, 1555);
        idealVelocityClose.add(71.93, 1560);
        idealVelocityClose.add(75.18, 1575);
        idealVelocityClose.add(79.21, 1620);
        idealVelocityClose.add(82.71, 1635);
        idealVelocityClose.add(85.16, 1650);
        idealVelocityClose.add(89.22, 1660);
        idealVelocityClose.add(93.14, 1680);
        idealVelocityClose.add(96.28, 1745);
        idealVelocityClose.add(100.91, 1840);


        // --- Hood Regression Data (close zone) ---
        // Distance: 36.53
        hoodRegressionClose.add(36.53, 1320, 0.04);
        hoodRegressionClose.add(36.53, 1345, 0.05);
        hoodRegressionClose.add(36.53, 1370, 0.07);

        //Distance: 40.27
        hoodRegressionClose.add(40.27, 1380, 0.13);
        hoodRegressionClose.add(40.27, 1420, 0.11);
        hoodRegressionClose.add(40.27, 1470, 0.1);

        // Distance: 43.46
        hoodRegressionClose.add(43.46, 1370, 0.15);
        hoodRegressionClose.add(43.46, 1445, 0.14);
        hoodRegressionClose.add(43.46, 1495, 0.10);

        //Distance: 49.462
        hoodRegressionClose.add(49.46, 1370, 0.17);
        hoodRegressionClose.add(49.46, 1420, 0.13);
        hoodRegressionClose.add(49.46, 1470, 0.1);

        // Distance: 51.22
        hoodRegressionClose.add(51.22, 1380, 0.17);
        hoodRegressionClose.add(51.22, 1420, 0.15);
        hoodRegressionClose.add(51.22, 1480, 0.13);

        //Distance: 54.29
        hoodRegressionClose.add(54.29, 1420, 0.15);
        hoodRegressionClose.add(54.29, 1470, 0.13);
        hoodRegressionClose.add(54.29, 1500, 0.11);

        // Distance: 57.55 
        hoodRegressionClose.add(57.55, 1420, 0.18);
        hoodRegressionClose.add(57.55, 1495, 0.20);
        hoodRegressionClose.add(57.55, 1545, 0.16);

        // Distance: 59.49 
        hoodRegressionClose.add(59.49, 1495, 0.14);
        hoodRegressionClose.add(59.49, 1545, 0.18);
        hoodRegressionClose.add(59.49, 1610, 0.17);

        // Distance: 62.22 
        hoodRegressionClose.add(62.22, 1480, 0.14);
        hoodRegressionClose.add(62.22, 1530, 0.18);
        hoodRegressionClose.add(62.22, 1540, 0.19);

        // Distance: 66.33 
        hoodRegressionClose.add(66.33, 1490, 0.14);
        hoodRegressionClose.add(66.33, 1540, 0.16);
        hoodRegressionClose.add(66.33, 1555, 0.17);

        // Distance: 71.93 de refacut
        //l-am refacut
        hoodRegressionClose.add(71.93, 1510, 0.13);
        hoodRegressionClose.add(71.93, 1545, 0.14);
        hoodRegressionClose.add(71.93, 1570, 0.15);

        // Distance: 75.18
        hoodRegressionClose.add(75.18, 1510, 0.13);
        hoodRegressionClose.add(75.18, 1540, 0.12);
        hoodRegressionClose.add(75.18, 1575, 0.16);

        // Distance: 79.21
        hoodRegressionClose.add(79.21, 1560, 0.16);
        hoodRegressionClose.add(79.21, 1580, 0.16);
        hoodRegressionClose.add(79.21, 1610, 0.16);

        // Distance: 82.71
        hoodRegressionClose.add(82.71, 1580, 0.14);
        hoodRegressionClose.add(82.71, 1620, 0.14);
        hoodRegressionClose.add(82.71, 1625, 0.23);

        // Distance: 85.16 
        hoodRegressionClose.add(85.16, 1590, 0.13);
        hoodRegressionClose.add(85.16, 1625, 0.24);
        hoodRegressionClose.add(85.16, 1650, 0.3);

        // Distance: 89.22 
        hoodRegressionClose.add(89.22, 1600, 0.16);
        hoodRegressionClose.add(89.22, 1630, 0.14);
        hoodRegressionClose.add(89.22, 1660, 0.14);

        // Distance: 93.14 
        hoodRegressionClose.add(93.14, 1630, 0.14);
        hoodRegressionClose.add(93.14, 1650, 0.13);
        hoodRegressionClose.add(93.14, 1680, 0.08);

        // Distance: 96.28 
        hoodRegressionClose.add(96.28, 1690, 0.08);
        hoodRegressionClose.add(96.28, 1720, 0.06);
        hoodRegressionClose.add(96.28, 1745, 0.25);

        // Distance: 100.91 
        hoodRegressionClose.add(100.91, 1770, 0.13);
        hoodRegressionClose.add(100.91, 1805, 0.1);
        hoodRegressionClose.add(100.91, 1840, 0.14);




        // --- Ideal Velocity Data (far zone) ---
        idealVelocityFar.add(36.53, 1370);
        idealVelocityFar.add(36.54, 1370);

        // --- Hood Regression Data (far zone) ---
        // Distance: 136.42
        hoodRegressionFar.add(136.42, 1,4 );
        hoodRegressionFar.add(136.42, 2, 5);
        hoodRegressionFar.add(136.42, 3, 6);

        // --- Build Tables ---
        idealVelocityClose.createLUT();
        idealVelocityFar.createLUT();
        hoodRegressionClose.create();
        hoodRegressionFar.create();
    }
    public static double target_tilt = 0.5;
    public static double power;

    public double getPower() {
        return power;
    }

    public void changeTarget(){
        // hook for manual overrides if needed
    }
    @Override
    public void update() {
        currentVel = motor1.getVelocity();
        changeTarget();
        double targetDistance = sensors.getDistanceToTarget(sensors.getTargetX(), sensors.getTargetY());
        double robotX = sensors.getX();

        // select which regression to use based on robot X
        boolean useFarZone = robotX > FAR_ZONE_X_THRESHOLD;
        InterpLUT activeVelocityLUT = useFarZone ? idealVelocityFar : idealVelocityClose;
        MultipleRegression activeHoodRegression = useFarZone ? hoodRegressionFar : hoodRegressionClose;

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
                        target = activeVelocityLUT.get(targetDistance);
                        target_tilt = activeHoodRegression.getHoodAngle(targetDistance, target);
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
                    target = auto_aim ? activeVelocityLUT.get(targetDistance) : target;
                } catch (Exception e) {
                    robot.op.gamepad1.rumble(250);
                    target = OuttakePositions.defaultVel;
                }
                power = pid.update(target, currentVel, sensors.getVoltage());
                motor1.setPower(power);
                motor2.setPower(power);
                break;
            case SPIN_UP:
                power = pid.update(target, currentVel, sensors.getVoltage());
                motor1.setPower(power);
                motor2.setPower(power);
                break;
            case LAUNCHING:
                if(auto_aim) {
                    try {
                        target = activeVelocityLUT.get(targetDistance);
                        target_tilt = activeHoodRegression.getHoodAngle(targetDistance, currentVel);
                    } catch (Exception e) {
                        robot.op.gamepad1.rumble(250);
                        robot.outtake.setOuttakeState(Outtake.OuttakeState.IDLE);
                        break;
                    }
                }
                // if auto_aim is false, again keep manual target & target_tilt
                power = pid.update(target, currentVel, sensors.getVoltage());
                power = pid.update(target, currentVel, sensors.getVoltage());
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
        return Math.abs(currentVel - target) < OuttakePositions.errorVelThreeshold;
    }
    public void autoAimOn(boolean on){
        auto_aim = on;
    }


}