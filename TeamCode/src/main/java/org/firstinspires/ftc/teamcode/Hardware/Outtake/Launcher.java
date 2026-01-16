package org.firstinspires.ftc.teamcode.Hardware.Outtake;


import static org.firstinspires.ftc.teamcode.Hardware.Outtake.OuttakePositions.FAR_ZONE_X_THRESHOLD;

import com.acmerobotics.dashboard.config.Config;
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
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setDirection(DcMotorEx.Direction.REVERSE);
        this.sensors = sensors;
    }

    public static double target_tilt = LauncherConstants.target_tilt;
    public static double power;

    public double getPower() {
        return power;
    }

    public void changeTarget(){
        // hook for manual overrides if needed
    }


    private double[] computePhysicsShot(double distanceIn) {
        distanceIn = Math.max(1e-3, distanceIn);

        double x = distanceIn;
        double y = LauncherConstants.TARGET_HEIGHT_DELTA_IN;

        double theta = Math.toRadians(35.0);

        for (int i = 0; i < 3; i++) {
            double tanT = Math.tan(theta);
            double cosT = Math.cos(theta);
            double denom = x * tanT - y;
            if (denom <= 0) {
                theta += Math.toRadians(5.0);
                theta = Range.clip(theta, LauncherConstants.MIN_LAUNCH_ANGLE_RAD, LauncherConstants.MAX_LAUNCH_ANGLE_RAD);
            } else {
                break;
            }
        }

        double tanT = Math.tan(theta);
        double cosT = Math.cos(theta);
        double denom = x * tanT - y;

        double v2;
        if (denom <= 0) {
            if (Math.abs(y) < 1e-3) {
                v2 = LauncherConstants.GRAVITY * x;
            } else {
                v2 = LauncherConstants.GRAVITY * x * x / (2.0 * Math.abs(y));
            }
        } else {
            v2 = LauncherConstants.GRAVITY * x * x / (2.0 * cosT * cosT * denom);
        }

        v2 = Math.max(v2, 0.0);
        double v = Math.sqrt(v2);

        return new double[]{v, theta};
    }

    private double projectileSpeedToTPS(double speedInPerSec) {
        double revPerSec = speedInPerSec / (2.0 * Math.PI * LauncherConstants.FLYWHEEL_RADIUS_IN);
        revPerSec = revPerSec / Math.max(LauncherConstants.FLYWHEEL_GEAR_RATIO, 1e-3);
        revPerSec *= LauncherConstants.PROJECTILE_SPEED_TO_RPM_SCALE;
        double tps = revPerSec * LauncherConstants.SHOOTER_TICKS_PER_REV;
        return Math.max(tps, 0.0);
    }

    private void updatePhysicsTargets(double targetDistance) {
        double[] shot = computePhysicsShot(targetDistance);
        double speedInPerSec = shot[0];
        double angleRad = shot[1];

        target = projectileSpeedToTPS(speedInPerSec);

        double angleDeg = Math.toDegrees(angleRad);
        target_tilt = Range.clip(LauncherConstants.HOOD_SERVO_SLOPE * angleDeg + LauncherConstants.HOOD_SERVO_OFFSET, 0.0, 1.0);
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
                //robot.outtake.turret.backlashEvet();
                break;
            case SHOOT_STARTED:
                //robot.outtake.turret.backlashYok();
                if(auto_aim){
                    try {
                        updatePhysicsTargets(targetDistance);
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
                        updatePhysicsTargets(targetDistance);
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
                        updatePhysicsTargets(targetDistance);
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


    public double getProjectileSpeedEstimate() {
        double tps = target;
        if (LauncherConstants.SHOOTER_TICKS_PER_REV <= 0) return 0.0;
        double motorRevPerSec = tps / LauncherConstants.SHOOTER_TICKS_PER_REV;
        double flywheelRevPerSec = motorRevPerSec * LauncherConstants.FLYWHEEL_GEAR_RATIO;
        double rimSpeedInPerSec = 2.0 * Math.PI * LauncherConstants.FLYWHEEL_RADIUS_IN * flywheelRevPerSec;
        double projSpeed = LauncherConstants.PROJECTILE_TRANSFER_COEFF * rimSpeedInPerSec;
        if (projSpeed < 0) projSpeed = 0.0;
        return projSpeed;
    }
}
