package org.firstinspires.ftc.teamcode.Hardware.Outtake;


import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Hardware.Intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.Hardware.Intake.IntakeTransfer;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.Sensors;

public class Outtake {
    Robot robot;
    public Launcher launcher;
    public Turret turret;

    public boolean shootingWhileMoving = false;

    public enum OuttakeState {
        OFF,
        IDLE,
        SLEEP,
        START_FEEDING_RAPID_FIRE,
        RAPID_FIRE,
        PRECISE_SHOOT_FEEDING,
        PRECISE_SHOOT,
        READY_FLYWHEEL,
        STOP
    }
    public OuttakeState outtakeState = OuttakeState.IDLE;
    public Outtake(Robot robot, Sensors sensors){
        this.robot = robot;
        turret = new Turret(robot,sensors);
        launcher = new Launcher(robot,sensors);

    }
    public void start_feed_rapid(double target,double hood) {
        launcher.setTarget(target,hood);
        outtakeState = OuttakeState.START_FEEDING_RAPID_FIRE;
    }

    public void
    start_feed_precise(double target,double hood) {
        launcher.setTarget(target,hood);
        outtakeState = OuttakeState.PRECISE_SHOOT_FEEDING;
    }
    public void update() {
        switch (outtakeState) {
            case OFF:
                break;
            case IDLE:
                launcher.launcherState = Launcher.LauncherState.OFF;
                break;
            case START_FEEDING_RAPID_FIRE:
                if(launcher.isReady()) {
                    robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.START_TRANSFER);
                    outtakeState = OuttakeState.RAPID_FIRE;
                }
                break;
            case RAPID_FIRE:
                launcher.launcherState = Launcher.LauncherState.LAUNCHING;
                break;
            case PRECISE_SHOOT_FEEDING:
                if(launcher.isReady()) {
                    Log.w("Debug shoot precise","a intrat launcher ready");
                    robot.intakeTransfer.rampState = IntakeTransfer.RampState.OPEN;
                    robot.intakeTransfer.setPowerForTime(IntakeConstants.preciseShotPower, IntakeConstants.preciseShotDelay);
                    outtakeState = OuttakeState.PRECISE_SHOOT;
                }
                break;
            case PRECISE_SHOOT:
                Log.w("Debug shoot precise","a intrat in case Precise shoot");
                if(robot.intakeTransfer.intakeState == IntakeTransfer.IntakeState.OFF_OPEN) {
                    outtakeState = OuttakeState.PRECISE_SHOOT_FEEDING;
                    Log.w("Debug shoot precise","se intoarce inapoi in feeding");
                }
                break;
            case READY_FLYWHEEL:
                launcher.launcherState = Launcher.LauncherState.READY_FLYWHEEL;
                break;
            case STOP:
                launcher.launcherState = Launcher.LauncherState.OFF;
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF);
                outtakeState = OuttakeState.IDLE;
                break;

        }

        launcher.update();
        turret.update();
    }

    public void flywheelSpin(double target) {
        launcher.setFlywheel(target);
    }
    public boolean isLaunching() {
        return outtakeState != OuttakeState.STOP && outtakeState != OuttakeState.IDLE;
    }

    public void setOuttakeState(OuttakeState outtakeState) {
        this.outtakeState = outtakeState;
    }

    public DcMotorEx getShooterMotor() {
        return launcher.motor1;
    }

    public boolean isShootingWhileMoving() {
        return shootingWhileMoving;
    }

    public void setShootingWhileMoving(boolean enabled) {
        shootingWhileMoving = enabled;
    }
}

