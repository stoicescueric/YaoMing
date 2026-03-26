package org.firstinspires.ftc.teamcode.Hardware.Outtake;


import static org.firstinspires.ftc.teamcode.Hardware.Outtake.OuttakePositions.FAR_ZONE_X_THRESHOLD;


import com.acmerobotics.dashboard.config.Config;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
import org.firstinspires.ftc.teamcode.Util.Controllers.velocityController;
import org.firstinspires.ftc.teamcode.Util.HardwareUtils;
import org.firstinspires.ftc.teamcode.Util.Math.MultipleRegression;
import org.firstinspires.ftc.teamcode.Util.Utils;
import org.firstinspires.ftc.teamcode.Util.Wrapper.InterpLUT;
import org.firstinspires.ftc.teamcode.Util.Wrapper.TelemetryUtil;

@Config
public  class Launcher implements Module {
    CachingDcMotorEx motor1,motor2;

    CachingServo tilt;

    public static double[] Distances = {1, 50, 58, 66,70.5, 74, 82, 90, 98,106,114,  130,135,140,145,150,155,160};
    // Corresponding Velocity values
    public static double[] velValues = {1300, 1300, 1310, 1350, 1430, 1450, 1510, 1560, 1700,1750,  1850,1850,1900,1950,2000,2040,2040};
    public static double[] hoodValues = {0.04, 0.04, 0.12, 0.15,0.19, 0.21, 0.23, 0.25, 0.3, 0.36, 0.37,   0.37, 0.38, 0.41, 0.42, 0.44,0.45,0.45};
            //


    InterpLUT velocity = new InterpLUT();
    InterpLUT hood = new InterpLUT();

    public enum LauncherState {
        OFF,
        SPIN_UP,
        SHOOT_STARTED,
        IDLE,
        LAUNCHING,
        GO_TO_VEL_HOOD,
        READY_FLYWHEEL,
        RECYCLE,
        TUNE_PID
    }


    Sensors sensors;
    public double target = 0;
    public  boolean auto_aim = true;
    public double currentVel = 0;
    public  double recycleVelocity = 500;
    public  double recycleTilt = 1;
    public LauncherState launcherState = LauncherState.OFF;
    public static boolean rebuildTables = false;

    DcMotorEx encoder;
    Robot robot;
    public Launcher(Robot robot, Sensors sensors) {
        this.robot = robot;
        this.motor1 = new CachingDcMotorEx(robot.hw.get(DcMotorEx.class,"shooter1"),0.001);
        this.motor2 = new CachingDcMotorEx(robot.hw.get(DcMotorEx.class,"shooter2"),0.001);
        tilt = new CachingServo(robot.hw.get(Servo.class,"tilt"));
        HardwareUtils.unlock(motor1);
        HardwareUtils.unlock(motor2);
        addData();
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(DcMotorEx.Direction.REVERSE);
        this.sensors = sensors;
        velocityController.reset();
    }


    public void goToSpecificValues(double vel,double hood) {
        target = vel;
        target_tilt = hood;
        launcherState = LauncherState.GO_TO_VEL_HOOD;
    }
    public void goToSpecificValues(Pose pose) {
        target = velocity.get(sensors.getDistanceFromPose(pose));
        target_tilt = hood.get(sensors.getDistanceFromPose(pose));
        launcherState = LauncherState.GO_TO_VEL_HOOD;
    }
    public void addData() {
        rebuildLutVelFromDashboard();
        rebuildLutHoodFromDashboard();

    }
    public void rebuildLutVelFromDashboard() {
        velocity = new InterpLUT();
        int len = Math.min(Distances.length, velValues.length);
        for (int i = 0; i < len; i++) {
            velocity.add(Distances[i], velValues[i]);
        }
        velocity.createLUT();
    }
    public void rebuildLutHoodFromDashboard() {
        hood = new InterpLUT();
        int len = Math.min(Distances.length, hoodValues.length);
        for (int i = 0; i < len; i++) {
            hood.add(Distances[i], hoodValues[i]);
        }
        hood.createLUT();
    }
    public static double target_tilt = 0.5;
    public double power;

    public double getHoodPosition() {
        return target_tilt;
    }

    public double getPower() {
        return power;
    }


    public double getTunePidTarget() {
        return tunePidTarget;
    }

    public static boolean useAdaptiveVel = true;
    public static double idleVelocityClose = 1530;
    public static double idleVelocityFar = 1950;
    public static double tunePidTarget = 0;
    @Override
    public void update() {
        if(rebuildTables) {
            rebuildLutVelFromDashboard();
            rebuildLutHoodFromDashboard();
            rebuildTables = false;
        }

        currentVel = -robot.blob.returnFrVelocity();

        double targetDistance = robot.sensors.getShooterDistanceToBackboard();

        switch (launcherState){
            case OFF:
                target = 0;
                power = velocityController.calculate(target, currentVel, sensors.getVoltage());
                motor1.setPower(power);
                motor2.setPower(power);
                break;
            case TUNE_PID:
                power = velocityController.calculate(tunePidTarget,currentVel,sensors.getVoltage());
                motor1.setPower(power);
                motor2.setPower(power);
                break;
            case IDLE:
                double speed;

                try{
                    if(useAdaptiveVel) {
                        target = velocity.get(Utils.minMaxClip(targetDistance,Distances[0],Distances[velValues.length-1]));
                    }else {
                        if(sensors.isFarZone()) {
                            target = idleVelocityFar;
                        }else {
                            target = idleVelocityClose;
                        }
                    }
                    target_tilt = hood.get(Utils.minMaxClip(targetDistance,Distances[0],Distances[hoodValues.length-1]));
                } catch (Exception e) {
                    target = OuttakePositions.defaultVel;
                    target_tilt = 0.3;
                }
                power = velocityController.calculate(target, currentVel,sensors.getVoltage()); // 0.75 ca sa nu stea la full power constant
                motor1.setPower(power);
                motor2.setPower(power);

                break;
            case SHOOT_STARTED:
                //robot.outtake.turret.backlashYok();
                if(auto_aim){
                    try {
                        target = velocity.get(targetDistance);
                        target_tilt = hood.get(targetDistance);
                    } catch (Exception e) {
                        robot.op.gamepad1.rumble(250);
                        robot.outtake.setOuttakeState(Outtake.OuttakeState.IDLE);
                        break;
                    }
                }
                launcherState = LauncherState.SPIN_UP;
                break;
            case GO_TO_VEL_HOOD:
                power = velocityController.calculate(target,currentVel,sensors.getVoltage());
                motor1.setPower(power);
                motor2.setPower(power);
                break;
            case READY_FLYWHEEL:
                try {
                    if (auto_aim) {
                        target = velocity.get(targetDistance);
                        target_tilt = hood.get(targetDistance);
                    }
                } catch (Exception e) {
                    robot.op.gamepad1.rumble(250);
                    target = OuttakePositions.defaultVel;
                    target_tilt = 0.5;
                }
                power = velocityController.calculate(target, currentVel, sensors.getVoltage());
                motor1.setPower(power);
                motor2.setPower(power);
                break;
            case SPIN_UP:
                power = velocityController.calculate(target, currentVel, sensors.getVoltage());
                motor1.setPower(power);
                motor2.setPower(power);
                if(auto_aim) {
                    try {
                        target_tilt = hood.get(targetDistance);
                    } catch (Exception e) {
                        break;
                    }
                }
                break;
            case LAUNCHING:
                power = velocityController.calculate(target, currentVel, sensors.getVoltage());
                motor1.setPower(power);
                motor2.setPower(power);
                break;
            case RECYCLE:
                target = recycleVelocity;
                target_tilt = recycleTilt;
                power = velocityController.calculate(target, currentVel, sensors.getVoltage());
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
        if(!auto_aim){
            this.target = target;
            target_tilt = hood_tilt;
        }
        launcherState = LauncherState.SHOOT_STARTED;
    }

    public void setFlywheel(double target) {
        this.target = target;
        launcherState = LauncherState.SPIN_UP;
    }

    public void setTargetTPS(double targetTPS){
        this.target=targetTPS;

    }
    public void setTargetHood(double hood_tilt){
        target_tilt = hood_tilt;
    }
    public void setAuto_aim(boolean state) {
        auto_aim = state;
    }

    public boolean  isReady(){
        return Math.abs(currentVel - target) < OuttakePositions.errorVelThreeshold;
    }
    public void autoAimOn(boolean on){
        auto_aim = on;
    }

    public double getTarget_tilt(){
        return target_tilt;
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
