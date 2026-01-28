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
    int gateCycleCounter = 0;

    public enum AutoStates {
        IDLE,
        GO_TO_SCORE_FROM_START,
        WAIT_SCORE_PRELOAD,
        GO_PICKUP2,
        GO_TO_SCORE2,
        WAIT_SCORE2,
        GO_GATE_PICKUP,
        GATE_PATH_DONE,
        WAIT_GATE_PICKUP,
        GO_SCORE_GATE_PICKUP,
        WAIT_SCORE_GATE_PICKUP,
        GO_PICKUP1,
        GO_TO_SCORE1,
        WAIT_SCORE_1,
        GO_TO_PARK,
        PARK,
        SLEEP
    }

    public AutoStates autoStates = AutoStates.IDLE;
    Close15Constants constants;

    @Override
    public void init() {
        robot = new Robot(this);
        constants = new Close15Constants();
        constants.buildPaths(robot.drive);
        robot.drive.setPose(constants.startPose);

        robot.outtake.turret.turretState = Turret.TurretState.TRACKING;
        robot.outtake.launcher.autoAimOn(true);
    }

    @Override
    public void start() {
        pathTimer = new Timer();
        gateCycleCounter = 0;
        setPathState(AutoStates.GO_TO_SCORE_FROM_START);
    }

    @Override
    public void loop() {
        switch (autoStates) {
            case IDLE:
                break;
            case GO_TO_SCORE_FROM_START:
                robot.drive.followPath(constants.scorePreload, true);
                robot.outtake.specificValues(constants.scorePose);
                setPathState(AutoStates.WAIT_SCORE_PRELOAD);
                break;
            case WAIT_SCORE_PRELOAD:
                if (robot.drive.isBusy() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                robot.outtake.start_feed_rapid(constants.getLauncherVelocity(), constants.getHoodPosition());
                sleep(constants.getShootingTime(), AutoStates.GO_PICKUP2);
                break;
            case GO_PICKUP2:
                robot.outtake.specificValues(constants.scorePose);
                robot.drive.followPath(constants.grabPickup2, constants.getMaxPower(), true);
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.INTAKE);
                setPathState(AutoStates.GO_TO_SCORE2);
                break;
            case GO_TO_SCORE2:
                if (robot.drive.isBusy() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                robot.drive.followPath(constants.scorePickup2, true);
                setPathState(AutoStates.WAIT_SCORE2);
                break;
            case WAIT_SCORE2:
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF);
                if (robot.drive.isBusy() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                robot.outtake.start_feed_rapid(constants.getLauncherVelocity(), constants.getHoodPosition());
                sleep(constants.getShootingTime(), AutoStates.GO_GATE_PICKUP);
                break;
            case GO_GATE_PICKUP:
                robot.outtake.specificValues(constants.scorePose);
                robot.drive.followPath(constants.grabGatePickup, constants.getMaxPower(), true);
                setPathState(AutoStates.GATE_PATH_DONE);
                break;
            case GATE_PATH_DONE:
                if (robot.drive.isBusy() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                else {
                    robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.INTAKE);
                    setPathState(AutoStates.WAIT_GATE_PICKUP);
                }
                break;
            case WAIT_GATE_PICKUP:
                if (robot.intakeTransfer.intakeState == IntakeTransfer.IntakeState.OFF || pathTimer.getElapsedTime() > constants.getFailSafePickupTime()) {
                    setPathState(AutoStates.GO_SCORE_GATE_PICKUP);
                }
                break;
            case GO_SCORE_GATE_PICKUP:
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF);
                robot.drive.followPath(constants.scoreGatePickup, true);
                setPathState(AutoStates.WAIT_SCORE_GATE_PICKUP);
                break;
            case WAIT_SCORE_GATE_PICKUP:
                if (robot.drive.isBusy() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                robot.outtake.start_feed_rapid(constants.getLauncherVelocity(), constants.getHoodPosition());
                gateCycleCounter++;
                if (gateCycleCounter < Close15Constants.gateCycleCount) {
                    sleep(constants.getShootingTime(), AutoStates.GO_GATE_PICKUP);
                } else {
                    sleep(constants.getShootingTime(), AutoStates.GO_PICKUP1);
                }
                break;
            case GO_PICKUP1:
                robot.outtake.setOuttakeState(Outtake.OuttakeState.IDLE);
                robot.drive.followPath(constants.grabPickUp1, constants.getMaxPower(), true);
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.INTAKE);
                setPathState(AutoStates.GO_TO_SCORE1);
                break;
            case GO_TO_SCORE1:
                if (robot.drive.isBusy() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                robot.outtake.specificValues(constants.scorePose);
                robot.drive.followPath(constants.scorePickup1, true);
                setPathState(AutoStates.WAIT_SCORE_1);
                break;
            case WAIT_SCORE_1:
                if (robot.drive.isBusy() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF);
                robot.outtake.start_feed_rapid(constants.getLauncherVelocity(), constants.getHoodPosition());
                sleep(constants.getShootingTime(), AutoStates.GO_TO_PARK);
                break;
            case GO_TO_PARK:
                robot.outtake.setOuttakeState(Outtake.OuttakeState.READY_FLYWHEEL);
                robot.drive.followPath(constants.goToPark, true);
                setPathState(AutoStates.PARK);
                break;
            case PARK:
                if (robot.drive.isBusy()) break;
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
