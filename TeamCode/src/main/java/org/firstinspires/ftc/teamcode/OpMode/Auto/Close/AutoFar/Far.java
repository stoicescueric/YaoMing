package org.firstinspires.ftc.teamcode.OpMode.Auto.Close.AutoFar;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware.Intake.IntakeTransfer;
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
        GO_TO_INTER_STACK,
        PICKUP_STACK,
        GO_TO_SCORE_STACK,
        TAKE_CYLE_HP,
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
    @Override
    public void loop() {
        switch (autoStates) {
            case IDLE:
                break;
            case GO_TO_PRELOAD:
                robot.blob.setTargetPosition(constants.shootingPose);
                if (!robot.blob.inPosition() && pathTimer.getElapsedTime() < constants.getDtFailsafe()) break;
                robot.outtake.start_feed_rapid(1000, 0.2);
                sleep(constants.getShootingTime(), AutoStates.GO_TO_HP);
                break;
            case SHOOT_PRELOAD:
                break;
            case GO_TO_HP:
                robot.outtake.outtakeState = Outtake.OuttakeState.IDLE;
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.INTAKE);
                robot.blob.setTargetPosition(constants.preload);

                if(robot.blob.progress > 0.6) robot.blob.maxPower = 0.8;
                if (!robot.blob.inPosition() && pathTimer.getElapsedTime() < constants.getDtFailsafe()) break;

                setPathState(AutoStates.GO_TO_SCORE_HP);
                break;
            case GO_TO_SCORE_HP:
                robot.blob.setTargetPosition(constants.shootingPose);
                if (!robot.blob.inPosition() && pathTimer.getElapsedTime() < constants.getDtFailsafe()) break;
                robot.outtake.start_feed_rapid(1000, 0.2);
                sleep(constants.getShootingTime(), AutoStates.GO_TO_INTER_STACK);
                break;
            case GO_TO_INTER_STACK:

                break;
            case PICKUP_STACK:
                break;
            case GO_TO_SCORE_STACK:
                break;
            case TAKE_CYLE_HP:
                break;
            case SCORE_CYCLE_HP:
                break;
            case SLEEP:
                if (System.currentTimeMillis() - startSleep > sleeptime) {
                    setPathState(nextState);
                }
                break;
        }
    }
    public void setPathState(AutoStates pState) {
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
