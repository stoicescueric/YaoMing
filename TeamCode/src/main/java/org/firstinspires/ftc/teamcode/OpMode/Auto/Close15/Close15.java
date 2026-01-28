package org.firstinspires.ftc.teamcode.OpMode.Auto.Close15;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware.Intake.IntakeTransfer;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Turret;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

public class Close15 extends OpMode {
    Robot robot;
    Timer pathTimer;

    public enum AutoStates {
        IDLE,
        GO_TO_SCORE_FROM_START,
        WAIT_SCORE_PRELOAD,
        GO_PICKUP1,
        GO_TO_SCORE1,
        WAIT_SCORE_1,
        GO_TO_SCORE2,
        WAIT_SCORE2,
        GO_PICKUP2,
        GO_PICKUP3,
        CLEAR,
        GO_TO_SCORE3,
        WAIT_SCORE3,
        GO_TO_PARK,
        PARK,
        SLEEP

    }

    public AutoStates autoStates = AutoStates.IDLE;
    CloseConstants constants;

    @Override
    public void init() {
        robot = new Robot(this);
        constants = new CloseConstants();
        constants.buildPaths(robot.drive);
        robot.drive.setPose(constants.startPose);


        robot.outtake.turret.turretState = Turret.TurretState.TRACKING;
        robot.outtake.launcher.autoAimOn(true);
    }

    @Override
    public void start() {
        pathTimer = new Timer();
        setPathState(AutoStates.GO_TO_SCORE_FROM_START);
    }

    @Override
    public void loop() {
        if (robot.sensors.areAllBeamsLowForTime(250)) {
            robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF);
        }
        switch (autoStates) {
            case IDLE:
                break;
            case GO_TO_SCORE_FROM_START:
                robot.outtake.setOuttakeState(Outtake.OuttakeState.IDLE);
                robot.drive.followPath(constants.scorePreload,true);
                robot.outtake.turret.setPosFixed(constants.getTurretPosition());
                setPathState(AutoStates.WAIT_SCORE_PRELOAD);
                break;
            case WAIT_SCORE_PRELOAD:
                if (robot.drive.isBusy() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                robot.outtake.start_feed_rapid(constants.getLauncherVelocity(), constants.getHoodPosition());
                sleep(constants.getShootingTime(), AutoStates.GO_PICKUP1);
                break;
            case GO_PICKUP1:
                robot.outtake.setOuttakeState(Outtake.OuttakeState.IDLE);

                robot.drive.followPath(constants.grabPickUp1, constants.getMaxPower(),true);
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.INTAKE);
                setPathState(AutoStates.CLEAR);
                break;
            case CLEAR :
                if (robot.drive.isBusy() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF);
                robot.drive.followPath(constants.goClear, constants.getMaxClearPower(), false);
                setPathState(AutoStates.GO_TO_SCORE1);
                break;
            case GO_TO_SCORE1:
                if (robot.drive.isBusy() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                robot.outtake.setOuttakeState(Outtake.OuttakeState.READY_FLYWHEEL);
                robot.drive.followPath(constants.scorePickup1, true);
                setPathState(AutoStates.WAIT_SCORE_1);
                break;
            case WAIT_SCORE_1:
                if (robot.drive.isBusy()) break;
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF);
                robot.outtake.start_feed_rapid(constants.getLauncherVelocity(), constants.getHoodPosition());
                sleep(constants.getShootingTime(),AutoStates.GO_PICKUP2);
                break;
            case GO_PICKUP2:
                robot.outtake.setOuttakeState(Outtake.OuttakeState.READY_FLYWHEEL);
                robot.drive.followPath(constants.grabPickup2, constants.getMaxPower(), true);
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.INTAKE);
                setPathState(AutoStates.GO_TO_SCORE2);
                break;
            case GO_TO_SCORE2:
                if (robot.drive.isBusy() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                robot.drive.followPath(constants.scorePickup2,true);
                setPathState(AutoStates.WAIT_SCORE2);
                break;
            case WAIT_SCORE2:
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF);

                if (robot.drive.isBusy()) break;
                robot.outtake.start_feed_rapid(constants.getLauncherVelocity(), constants.getHoodPosition());
                sleep(constants.getShootingTime(), AutoStates.GO_PICKUP3);
                break;
            case GO_PICKUP3:
                robot.outtake.setOuttakeState(Outtake.OuttakeState.READY_FLYWHEEL);
                robot.drive.followPath(constants.grabPickup3, constants.getMaxPower(), true);
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.INTAKE);
                setPathState(AutoStates.GO_TO_SCORE3);
                break;
            case GO_TO_SCORE3:
                if (robot.drive.isBusy() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                robot.drive.followPath(constants.scorePickup3,true);
                setPathState(AutoStates.WAIT_SCORE3);
                break;
            case WAIT_SCORE3:
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF);
                if (robot.drive.isBusy()) break;
                robot.outtake.start_feed_rapid(constants.getLauncherVelocity(), constants.getHoodPosition());
                sleep(constants.getShootingTime(), AutoStates.GO_TO_PARK);
                break;
            case GO_TO_PARK:
                robot.outtake.setOuttakeState(Outtake.OuttakeState.READY_FLYWHEEL);
                robot.drive.followPath(constants.goToPark, true);
                setPathState(AutoStates.PARK);
                break;
            case PARK:
                if(robot.drive.isBusy()) break;
                robot.outtake.setOuttakeState(Outtake.OuttakeState.READY_FLYWHEEL);

                requestOpModeStop();
                break;
            case SLEEP:
                if (System.currentTimeMillis() - startSleep > sleeptime) {
                    setPathState(nextState);
                }
                break;
        }
        robot.update();
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
    public void setPathState(AutoStates pState) {
        autoStates = pState;
        pathTimer.resetTimer();
    }
}
