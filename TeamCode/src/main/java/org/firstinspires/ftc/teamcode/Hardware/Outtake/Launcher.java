package org.firstinspires.ftc.teamcode.Hardware.Outtake;


import static org.firstinspires.ftc.teamcode.Hardware.Outtake.OuttakePositions.FAR_ZONE_X_THRESHOLD;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware.Module;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.Sensors;
import org.firstinspires.ftc.teamcode.Util.Caching.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.Util.Caching.CachingServo;
import org.firstinspires.ftc.teamcode.Util.Controllers.FlyWheelPID;
import org.firstinspires.ftc.teamcode.Util.HardwareUtils;
import org.firstinspires.ftc.teamcode.Util.Math.MultipleRegression;
import org.firstinspires.ftc.teamcode.Util.Wrapper.TelemetryUtil;

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
        IDLE,
        LAUNCHING,
        READY_FLYWHEEL,
        RECYCLE
    }


    Sensors sensors;
    public double target = 0;
    public static boolean auto_aim = true;
    public static double idleAlpha = 0.5;
    public double currentVel = 0;
    public static double recycleVelocity = 500;
    public static double recycleTilt = 1;
    public LauncherState launcherState = LauncherState.OFF;
    public FlyWheelPID pid = new FlyWheelPID();
    Robot robot;
    public Launcher(Robot robot, Sensors sensors) {
        this.robot = robot;
        this.motor1 = new CachingDcMotorEx(robot.hw.get(DcMotorEx.class,"shooter1"),0);
        this.motor2 = new CachingDcMotorEx(robot.hw.get(DcMotorEx.class,"shooter2"),0);

        tilt = new CachingServo(robot.hw.get(Servo.class,"hood"));
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
        idealVelocity.add(49.46, 1330);
        idealVelocity.add(51.27, 1330);
        idealVelocity.add(54.23, 1350);
        idealVelocity.add(57.55, 1350);
        idealVelocity.add(59.36, 1390);
        idealVelocity.add(62.22, 1390);
        idealVelocity.add(64.46, 1410);
        idealVelocity.add(66.33, 1370);
        idealVelocity.add(68.60, 1410);
        idealVelocity.add(71.80, 1420);
        idealVelocity.add(75.18, 1450);
        idealVelocity.add(77.92, 1480);
        idealVelocity.add(79.21, 1480);
        idealVelocity.add(81.50, 1515);
        idealVelocity.add(82.71, 1530);
        idealVelocity.add(85.16, 1535);
        idealVelocity.add(87.22, 1575);
        idealVelocity.add(91.14, 1620);
        idealVelocity.add(94.28, 1660);
        idealVelocity.add(98.91, 1710);
        idealVelocity.add(100.91, 1730);

        idealVelocity.add(121.84, 1833);
        idealVelocity.add(125.26, 1833);
        idealVelocity.add(128.02, 1833);
        idealVelocity.add(130.20, 1833);
        idealVelocity.add(133.19, 1833);
        idealVelocity.add(137.59, 1833);
        idealVelocity.add(141.04, 1833);
        idealVelocity.add(144.30, 1833);
        idealVelocity.add(146.96, 1833);
        idealVelocity.add(152.44, 1833);
        idealVelocity.add(154.13, 1833);


        // --- Hood Regression Data (close zone) ---

        // Distance: 49.46
        hoodRegression.add(41.46, 1270, 0.05);
        hoodRegression.add(41.46, 1290, 0.06);
        hoodRegression.add(41.46, 1330, 0.09);

        // Distance: 51.27
        hoodRegression.add(51.27, 1280, 0.07);
        hoodRegression.add(51.27, 1310, 0.08);
        hoodRegression.add(51.27, 1330, 0.10);

        // Distance: 54.23
        hoodRegression.add(54.23, 1310, 0.11);
        hoodRegression.add(54.23, 1330, 0.12);
        hoodRegression.add(54.23, 1350, 0.16);

        // Distance: 57.55
        hoodRegression.add(57.55, 1290, 0.16);
        hoodRegression.add(57.55, 1320, 0.17);
        hoodRegression.add(57.55, 1350, 0.20);

        // Distance: 59.36
        hoodRegression.add(59.36, 1340, 0.17);
        hoodRegression.add(59.36, 1365, 0.15);
        hoodRegression.add(59.36, 1390, 0.1875); //P.s schema ca sa o suga ericoi cioroi!!!

        // Distance: 62.22
        hoodRegression.add(62.22, 1320, 0.18);
        hoodRegression.add(62.22, 1340, 0.20);
        hoodRegression.add(62.22, 1390, 0.24);

        // Distance: 64.46
        hoodRegression.add(64.46, 1350, 0.195);
        hoodRegression.add(64.46, 1390, 0.22);
        hoodRegression.add(64.46, 1410, 0.255);

        // Distance: 66.33
        hoodRegression.add(66.33, 1320, 0.19);
        hoodRegression.add(66.33, 1350, 0.22);
        hoodRegression.add(66.33, 1370, 0.25);

        // Distance: 68.60
        hoodRegression.add(68.60, 1360, 0.225);
        hoodRegression.add(68.60, 1390, 0.24);
        hoodRegression.add(68.60, 1410, 0.26);

        // Distance: 71.80
        hoodRegression.add(71.80, 1370, 0.20);
        hoodRegression.add(71.80, 1400, 0.20);
        hoodRegression.add(71.80, 1420, 0.22); //420 hehehehhehehehehhehehehehhe nice

        // Distance: 75.18
        hoodRegression.add(75.18, 1410, 0.21);
        hoodRegression.add(75.18, 1430, 0.22);
        hoodRegression.add(75.18, 1450, 0.26);

        // Distance: 77.92
        hoodRegression.add(77.92, 1435, 0.23);
        hoodRegression.add(77.92, 1460, 0.245);
        hoodRegression.add(77.92, 1480,0.255);

        // Distance: 79.21
        hoodRegression.add(79.21, 1430, 0.23);
        hoodRegression.add(79.21, 1455, 0.26);
        hoodRegression.add(79.21, 1480, 0.29);

        // Distance: 81.50
        hoodRegression.add(81.50, 1450, 0.265);
        hoodRegression.add(81.50, 1480, 0.27);
        hoodRegression.add(81.50, 1515, 0.28);

        // Distance: 82.71
        hoodRegression.add(82.71, 1470, 0.28);
        hoodRegression.add(82.71, 1490, 0.32);
        hoodRegression.add(82.71, 1530, 0.38); //diferenta este din pid nu din valoare

        // Distance: 85.16
        hoodRegression.add(85.16, 1490, 0.34);
        hoodRegression.add(85.16, 1510, 0.37);
        hoodRegression.add(85.16, 1535, 0.39);

        // Distance: 87.22
        hoodRegression.add(87.22, 1510, 0.36);
        hoodRegression.add(87.22, 1540, 0.375);
        hoodRegression.add(87.22, 1575, 0.395);

        // Distance: 91.14
        hoodRegression.add(91.14, 1555, 0.365);
        hoodRegression.add(91.14, 1590, 0.355);
        hoodRegression.add(91.14, 1620, 0.39);

        // Distance: 94.28
        hoodRegression.add(94.28, 1610, 0.37);
        hoodRegression.add(94.28, 1630, 0.42);
        hoodRegression.add(94.28, 1660, 0.46);

        // Distance: 98.91
        hoodRegression.add(98.91, 1630, 0.39);
        hoodRegression.add(98.91, 1670, 0.45);
        hoodRegression.add(98.91, 1710, 0.48);

        // Distance: 100.91
        hoodRegression.add(100.91, 1680, 0.45);
        hoodRegression.add(100.91, 1710, 0.48);
        hoodRegression.add(100.91, 1730, 0.495);


        // --- Hood Regression Data (far zone) ---
        // Distance: 121.84
// Distance: 121.84
        hoodRegression.add(121.84, 1803, 0.350);
        hoodRegression.add(121.84, 1823, 0.350);
        hoodRegression.add(121.84, 1833, 0.345);

// Distance: 125.26
        hoodRegression.add(125.26, 1803, 0.350);
        hoodRegression.add(125.26, 1823, 0.350);
        hoodRegression.add(125.26, 1833, 0.345);

// Distance: 128.02
        hoodRegression.add(128.02, 1803, 0.350);
        hoodRegression.add(128.02, 1823, 0.350);
        hoodRegression.add(128.02, 1833, 0.345);

// Distance: 130.20
        hoodRegression.add(130.20, 1803, 0.350);
        hoodRegression.add(130.20, 1823, 0.350);
        hoodRegression.add(130.20, 1833, 0.345);

// Distance: 133.19
        hoodRegression.add(133.19, 1803, 0.350);
        hoodRegression.add(133.19, 1823, 0.350);
        hoodRegression.add(133.19, 1833, 0.345);

// Distance: 137.59
        hoodRegression.add(137.59, 1803, 0.350);
        hoodRegression.add(137.59, 1823, 0.350);
        hoodRegression.add(137.59, 1833, 0.345);

// Distance: 141.04
        hoodRegression.add(141.04, 1803, 0.350);
        hoodRegression.add(141.04, 1823, 0.350);
        hoodRegression.add(141.04, 1833, 0.345);

// Distance: 144.30
        hoodRegression.add(144.30, 1803, 0.350);
        hoodRegression.add(144.30, 1823, 0.350);
        hoodRegression.add(144.30, 1833, 0.345);

// Distance: 146.96
        hoodRegression.add(146.96, 1803, 0.350);
        hoodRegression.add(146.96, 1823, 0.350);
        hoodRegression.add(146.96, 1833, 0.345);

// Distance: 152.44
        hoodRegression.add(152.44, 1803, 0.350);
        hoodRegression.add(152.44, 1823, 0.350);
        hoodRegression.add(152.44, 1833, 0.345);

// Distance: 154.13
        hoodRegression.add(154.13, 1803, 0.350);
        hoodRegression.add(154.13, 1823, 0.350);
        hoodRegression.add(154.13, 1833, 0.345);
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
        currentVel = motor2.getVelocity();
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
            case IDLE:
                power = pid.update(target*idleAlpha, currentVel, sensors.getVoltage()); // 0.75 ca sa nu stea la full power constant
                motor1.setPower(power);
                motor2.setPower(power);
                break;
            case SHOOT_STARTED:
                //robot.outtake.turret.backlashYok();
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
            case RECYCLE:
                target = recycleVelocity;
                target_tilt = recycleTilt;
                power = pid.update(target, currentVel, sensors.getVoltage());
                motor1.setPower(power);
                motor2.setPower(power);
                break;
        }
        tilt.setPosition(target_tilt);
        TelemetryUtil.packet.put("shooter1  amps",motor1.getCurrent(CurrentUnit.AMPS));
        TelemetryUtil.packet.put("shooter2 amps",motor2.getCurrent(CurrentUnit.AMPS));
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
    public static double FLYWHEEL_RADIUS_IN = 69.2 / 25.4;
    public static double FLYWHEEL_GEAR_RATIO = 1.0;
    public static double PROJECTILE_TRANSFER_COEFF = 0.7; // tune in dashboard


    public double getProjectileSpeedEstimate() {
        double tps = target;
        if (SHOOTER_TICKS_PER_REV <= 0) return 0.0;
        double motorRevPerSec = tps / SHOOTER_TICKS_PER_REV;
        double flywheelRevPerSec = motorRevPerSec * FLYWHEEL_GEAR_RATIO;
        double rimSpeedInPerSec = 2.0 * Math.PI * FLYWHEEL_RADIUS_IN * flywheelRevPerSec;
        double projSpeed = PROJECTILE_TRANSFER_COEFF * rimSpeedInPerSec;
        if (projSpeed < 0) projSpeed = 0.0;
        return projSpeed;
    }
}
