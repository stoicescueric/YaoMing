package org.firstinspires.ftc.teamcode.Hardware.Outtake;


import static org.firstinspires.ftc.teamcode.Hardware.Outtake.OuttakePositions.FAR_ZONE_X_THRESHOLD;

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

        // --- Ideal Velocity Data (unified close + far) ---
        idealVelocity.add(36.53, 1410);
        idealVelocity.add(43.46, 1515);
        idealVelocity.add(51.22, 1520);
        idealVelocity.add(57.55, 1565);
        idealVelocity.add(59.49, 1640);
        idealVelocity.add(62.22, 1580);
        idealVelocity.add(66.33, 1595);
        idealVelocity.add(71.93, 1600);
        idealVelocity.add(75.18, 1615);
        idealVelocity.add(79.21, 1660);
        idealVelocity.add(82.71, 1675);
        idealVelocity.add(85.16, 1690);
        idealVelocity.add(89.22, 1700);
        idealVelocity.add(93.14, 1720);
        idealVelocity.add(96.28, 1785);
        idealVelocity.add(100.91, 1880);

        idealVelocity.add(121.84, 1865);
        idealVelocity.add(125.26, 1890);
        idealVelocity.add(128.02, 1930);
        idealVelocity.add(130.20, 1940);
        idealVelocity.add(133.19, 1970);
        idealVelocity.add(137.59, 2070);
        idealVelocity.add(141.04, 2080);
        idealVelocity.add(144.30, 2090);
        idealVelocity.add(146.96, 2120);
        idealVelocity.add(152.44, 2120);
        idealVelocity.add(154.13, 2160);


        // --- Hood Regression Data (close zone) ---
        // Distance: 36.53
        hoodRegression.add(36.53, 1360, 0.17);
        hoodRegression.add(36.53, 1385, 0.18);
        hoodRegression.add(36.53, 1410, 0.20);

        // Distance: 40.27
        hoodRegression.add(40.27, 1420, 0.26);
        hoodRegression.add(40.27, 1460, 0.24);
        hoodRegression.add(40.27, 1510, 0.23);

        // Distance: 43.46
        hoodRegression.add(43.46, 1410, 0.28);
        hoodRegression.add(43.46, 1485, 0.27);
        hoodRegression.add(43.46, 1535, 0.23);

        // Distance: 49.46
        hoodRegression.add(49.46, 1410, 0.30);
        hoodRegression.add(49.46, 1460, 0.26);
        hoodRegression.add(49.46, 1510, 0.23);

        // Distance: 51.22
        hoodRegression.add(51.22, 1420, 0.24);
        hoodRegression.add(51.22, 1460, 0.24);
        hoodRegression.add(51.22, 1520, 0.23);

        // Distance: 54.29
        hoodRegression.add(54.29, 1460, 0.25);
        hoodRegression.add(54.29, 1510, 0.25);
        hoodRegression.add(54.29, 1540, 0.25);

        // Distance: 57.55
        hoodRegression.add(57.55, 1460, 0.31);
        hoodRegression.add(57.55, 1535, 0.33);
        hoodRegression.add(57.55, 1585, 0.29);

        // Distance: 59.49
        hoodRegression.add(59.49, 1535, 0.27);
        hoodRegression.add(59.49, 1585, 0.31);
        hoodRegression.add(59.49, 1650, 0.30);

        // Distance: 62.22
        hoodRegression.add(62.22, 1520, 0.27);
        hoodRegression.add(62.22, 1570, 0.31);
        hoodRegression.add(62.22, 1580, 0.32);

        // Distance: 66.33
        hoodRegression.add(66.33, 1530, 0.27);
        hoodRegression.add(66.33, 1580, 0.29);
        hoodRegression.add(66.33, 1595, 0.30);

        // Distance: 71.93
        hoodRegression.add(71.93, 1550, 0.26);
        hoodRegression.add(71.93, 1585, 0.27);
        hoodRegression.add(71.93, 1610, 0.28);

        // Distance: 75.18
        hoodRegression.add(75.18, 1550, 0.26);
        hoodRegression.add(75.18, 1580, 0.25);
        hoodRegression.add(75.18, 1615, 0.29);

        // Distance: 79.21
        hoodRegression.add(79.21, 1590, 0.29);
        hoodRegression.add(79.21, 1610, 0.29);
        hoodRegression.add(79.21, 1640, 0.29);

        // Distance: 82.71
        hoodRegression.add(82.71, 1610, 0.27);
        hoodRegression.add(82.71, 1650, 0.27);
        hoodRegression.add(82.71, 1655, 0.36);

        // Distance: 85.16
        hoodRegression.add(85.16, 1620, 0.26);
        hoodRegression.add(85.16, 1655, 0.37);
        hoodRegression.add(85.16, 1690, 0.43);

        // Distance: 89.22
        hoodRegression.add(89.22, 1630, 0.29);
        hoodRegression.add(89.22, 1660, 0.27);
        hoodRegression.add(89.22, 1690, 0.27);

        // Distance: 93.14
        hoodRegression.add(93.14, 1670, 0.30);
        hoodRegression.add(93.14, 1690, 0.29);
        hoodRegression.add(93.14, 1720, 0.30);

        // Distance: 96.28
        hoodRegression.add(96.28, 1730, 0.24);
        hoodRegression.add(96.28, 1760, 0.23);
        hoodRegression.add(96.28, 1785, 0.40);

        // Distance: 100.91
        hoodRegression.add(100.91, 1810, 0.32);
        hoodRegression.add(100.91, 1845, 0.30);
        hoodRegression.add(100.91, 1880, 0.34);

        // --- Hood Regression Data (far zone) ---
        // Distance: 121.84
        hoodRegression.add(121.84, 1835, 0.22);
        hoodRegression.add(121.84, 1850, 0.21);
        hoodRegression.add(121.84, 1865, 0.22);

        // Distance: 125.26
        hoodRegression.add(125.26, 1860, 0.21);
        hoodRegression.add(125.26, 1875, 0.22);
        hoodRegression.add(125.26, 1890, 0.22);

        // Distance: 128.02
        hoodRegression.add(128.02, 1880, 0.20);
        hoodRegression.add(128.02, 1910, 0.20);
        hoodRegression.add(128.02, 1930, 0.22);

        // Distance: 130.20
        hoodRegression.add(130.20, 1910, 0.24);
        hoodRegression.add(130.20, 1930, 0.25);
        hoodRegression.add(130.20, 1940, 0.27);

        // Distance: 133.19
        hoodRegression.add(133.19, 1950, 0.26);
        hoodRegression.add(133.19, 1960, 0.32);
        hoodRegression.add(133.19, 1970, 0.34);

        // Distance: 137.59
        hoodRegression.add(137.59, 2030, 0.40);
        hoodRegression.add(137.59, 2050, 0.44);
        hoodRegression.add(137.59, 2070, 0.46);

        // Distance: 141.04
        hoodRegression.add(141.04, 2040, 0.40);
        hoodRegression.add(141.04, 2060, 0.44);
        hoodRegression.add(141.04, 2080, 0.46);

        // Distance: 144.30
        hoodRegression.add(144.30, 2050, 0.42);
        hoodRegression.add(144.30, 2065, 0.45);
        hoodRegression.add(144.30, 2090, 0.48);

        // Distance: 146.96
        hoodRegression.add(146.96, 2070, 0.41);
        hoodRegression.add(146.96, 2090, 0.40);
        hoodRegression.add(146.96, 2120, 0.40);

        // Distance: 152.44
        hoodRegression.add(152.44, 2090, 0.43);
        hoodRegression.add(152.44, 2100, 0.42);
        hoodRegression.add(152.44, 2120, 0.41);

        // Distance: 154.13
        hoodRegression.add(154.13, 2120, 0.39);
        hoodRegression.add(154.13, 2140, 0.44);
        hoodRegression.add(154.13, 2160, 0.46);

        idealVelocity.createLUT();
        hoodRegression.create();
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

        switch (launcherState){
            case OFF:
                target = 0;
                power = pid.update(target, currentVel, sensors.getVoltage());
                motor1.setPower(power);
                motor2.setPower(power);
                robot.outtake.turret.backlashEvet();
                break;
            case SHOOT_STARTED:
                robot.outtake.turret.backlashYok();
                if(auto_aim){
                    try {
                        target = idealVelocity.get(targetDistance);
                        target_tilt = hoodRegression.getHoodAngle(targetDistance, target);
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
                    if (auto_aim) {
                        target = idealVelocity.get(targetDistance);
                    }
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
                        target = idealVelocity.get(targetDistance);
                        target_tilt = hoodRegression.getHoodAngle(targetDistance, currentVel);
                    } catch (Exception e) {
                        robot.op.gamepad1.rumble(250);
                        robot.outtake.setOuttakeState(Outtake.OuttakeState.IDLE);
                        break;
                    }
                }
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

    public static double SHOOTER_TICKS_PER_REV = 28.0;
    // Flywheel radius in inches (69.2 mm -> 2.7244 inches).
    public static double FLYWHEEL_RADIUS_IN = 69.2 / 25.4;
    // Gear ratio from motor to flywheel (1:1 by user description).
    public static double FLYWHEEL_GEAR_RATIO = 1.0;
    // Effective transfer coefficient from rim speed to projectile speed.
    public static double PROJECTILE_TRANSFER_COEFF = 0.7; // tune in dashboard

    /**
     * Estimates projectile linear speed (inches/second) based on the current
     * launcher target TPS (ticks per second) and the physical flywheel
     * parameters. This is used for time-of-flight in motion compensation.
     */
    public double getProjectileSpeedEstimate() {
        double tps = target; // target ticks per second the PID is chasing
        if (SHOOTER_TICKS_PER_REV <= 0) return 0.0;

        // Convert motor ticks/s -> motor rev/s
        double motorRevPerSec = tps / SHOOTER_TICKS_PER_REV;

        // Apply gear ratio to get flywheel rev/s
        double flywheelRevPerSec = motorRevPerSec * FLYWHEEL_GEAR_RATIO;

        // Rim linear speed: v = 2πr * rev/s
        double rimSpeedInPerSec = 2.0 * Math.PI * FLYWHEEL_RADIUS_IN * flywheelRevPerSec;

        // Effective projectile speed: some fraction of rim speed
        double projSpeed = PROJECTILE_TRANSFER_COEFF * rimSpeedInPerSec;
        if (projSpeed < 0) projSpeed = 0.0;
        return projSpeed;
    }
}




