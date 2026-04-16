package org.firstinspires.ftc.teamcode.OpMode.Auto.Close.AutoFar;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware.Intake.IntakeTransfer;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Launcher;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Turret;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.OpMode.Auto.Close.Close18.Close18;
import org.firstinspires.ftc.teamcode.Util.Globals.Phase;
import org.firstinspires.ftc.teamcode.Util.Info;

public class Far extends OpMode {
    Robot robot;
    ConstantsFar constants;
    public enum AutoStates {
        IDLE,
        GO_TO_PRELOAD,
        SHOOT_PRELOAD,
        GO_TO_HP,
        GO_TO_SCORE_HP,
        SCORE_HP,
        GO_TO_INTER_STACK,
        PICKUP_STACK,
        GO_TO_SCORE_STACK,
        TAKE_CYLE_HP,
        PARK,
        SCORE_CYCLE_HP,
        SLEEP
    }

    public AutoStates autoStates = AutoStates.IDLE;
    double preloadCycle = 0;
    Timer pathTimer;

    @Override
    public void init() {
        Info.phase = Phase.AUTONOMOUS;
        Info.useBlob = true;

        robot = new Robot(this);
        constants = new ConstantsFar();
        robot.outtake.turret.turretState = Turret.TurretState.TRACKING;
        robot.outtake.launcher.autoAimOn(true);
        robot.outtake.outtakeState = Outtake.OuttakeState.IDLE;
        robot.intakeTransfer.intakeState = IntakeTransfer.IntakeState.OFF_OPEN;
    }

    @Override
    public void start() {
        robot.blob.odo.setPose(constants.startPose);
        robot.blob.odo.update();
        constants.buildPath();
        preloadCycle = 0;
        pathTimer = new Timer();
        setPathState(AutoStates.GO_TO_PRELOAD);
    }
    int cntCycle = 0;
    boolean newState = false;
    @Override
    public void loop() {
        switch (autoStates) {
            case IDLE:
                break;
            case GO_TO_PRELOAD:
                robot.blob.setTargetPosition(constants.shootingPose);
                if (!robot.blob.inPosition() && pathTimer.getElapsedTime() < constants.getDtFailsafe()) break;
                robot.outtake.start_feed_rapid(1000, 0.2);
                autoStates = AutoStates.SHOOT_PRELOAD;
                break;
            case SHOOT_PRELOAD:
                if(robot.outtake.launcher.launcherState == Launcher.LauncherState.LAUNCHING) {

                    sleep(constants.getShootingTime(), AutoStates.GO_TO_HP);
                }
                break;
            case GO_TO_HP:
                robot.outtake.outtakeState = Outtake.OuttakeState.IDLE;

                robot.blob.setTargetPosition(constants.preload);

                if(robot.blob.progress > 0.6){
                    robot.blob.maxPower = 0.8;
                    robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.INTAKE);
                }
                if (!robot.blob.inPosition() && pathTimer.getElapsedTime() < constants.getDtFailsafe() && robot.blob.isStuck()) break;

                setPathState(AutoStates.GO_TO_SCORE_HP);
                break;
            case GO_TO_SCORE_HP:
                robot.blob.setTargetPosition(constants.shootingPose);
                if (!robot.blob.inPosition() && pathTimer.getElapsedTime() < constants.getDtFailsafe()) break;
                robot.outtake.start_feed_rapid(1000, 0.2);
                autoStates= AutoStates.SCORE_HP;
                break;
            case SCORE_HP:
                if(robot.outtake.launcher.launcherState == Launcher.LauncherState.LAUNCHING) {
                    ++cntCycle;
                    if(cntCycle == constants.getCycles()) {
                        sleep(constants.getShootingTime(), AutoStates.PARK);
                    }else {
                        sleep(constants.getShootingTime(), AutoStates.GO_TO_HP);
                    }
                }
                break;
            case PARK:
                robot.blob.setTargetPosition(constants.park);
                if (!robot.blob.inPosition() && pathTimer.getElapsedTime() < constants.getDtFailsafe()) {
                    break;
                }
                requestOpModeStop();
                break;
            case SLEEP:
                if (System.currentTimeMillis() - startSleep > sleeptime) {
                    setPathState(nextState);
                }
                break;
        }
    }
    public void setPathState(AutoStates pState) {
        newState = true;
        autoStates = pState;
        pathTimer.resetTimer();
    }
    long startSleep = 0;
    double sleeptime = 0;
    AutoStates nextState = AutoStates.IDLE;
    private void sleep(double time, AutoStates nextState) {
        startSleep = System.currentTimeMillis();
        setPathState(AutoStates.SLEEP);
        sleeptime = time;
        this.nextState = nextState;
    }
}
