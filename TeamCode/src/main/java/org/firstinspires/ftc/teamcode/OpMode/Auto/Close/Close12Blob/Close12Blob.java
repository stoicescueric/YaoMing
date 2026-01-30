package org.firstinspires.ftc.teamcode.OpMode.Auto.Close.Close12Blob;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware.Intake.IntakeTransfer;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Turret;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Util.Globals.Phase;
import org.firstinspires.ftc.teamcode.Util.Info;
import org.firstinspires.ftc.teamcode.Util.Wrapper.TelemetryUtil;

public class Close12Blob extends OpMode {
    Robot robot;
    Timer pathTimer;

    public enum AutoStates {
        IDLE,
        GO_TO_SCORE_FROM_START,
        WAIT_SCORE_PRELOAD,
        GO_PICKUP1,
        GO_CLEAR,
        GO_CLEAR_INTER,
        GO_TO_SCORE1,
        WAIT_SCORE_1,
        GO_PICKUP2,
        GO_PICKUP2_INTERMEDIARY,
        GO_TO_SCORE2,
        WAIT_SCORE2,
        GO_PICKUP3,
        GO_PICKUP3_INTERMEDIARY,
        GO_TO_SCORE3,
        WAIT_SCORE3,
        GO_TO_PARK,
        PARK,
        SLEEP

    }

    public AutoStates autoStates = AutoStates.IDLE;
    CloseConstants12Blob constants;

    @Override
    public void init() {
        Info.phase = Phase.AUTONOMOUS;
        Info.useBlob = true;
        robot = new Robot(this);
        constants = new CloseConstants12Blob();
        constants.buildPaths(robot.drive);
        robot.outtake.turret.turretState = Turret.TurretState.TRACKING;
        robot.outtake.launcher.autoAimOn(true);
    }

    @Override
    public void start() {
        robot.blob.odo.setPose(constants.startPose);
        robot.blob.odo.update();
        pathTimer = new Timer();
        setPathState(AutoStates.GO_TO_SCORE_FROM_START);
    }

    @Override
    public void loop() {
        switch (autoStates) {
            case IDLE:
                break;
            case GO_TO_SCORE_FROM_START:
                robot.blob.setTargetPosition(constants.scorePose);
                robot.outtake.specificValues(constants.scorePose);
                setPathState(AutoStates.WAIT_SCORE_PRELOAD);
                break;
            case WAIT_SCORE_PRELOAD:
                if (!robot.blob.inPosition()) break;
                robot.outtake.start_feed_rapid(constants.getLauncherVelocity(), constants.getHoodPosition());
                sleep(constants.getShootingTime(), AutoStates.GO_PICKUP1);
                break;

            case GO_PICKUP1:
                robot.outtake.setOuttakeState(Outtake.OuttakeState.IDLE);
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.INTAKE);
                robot.blob.setTargetPosition(constants.pickUpPose);
                if (!robot.blob.inPosition() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                robot.blob.setTargetPosition(constants.clearInter);
                setPathState(AutoStates.GO_CLEAR);
                break;
            case GO_CLEAR_INTER:
                robot.blob.setTargetPosition(constants.clearInter);
                if (!robot.blob.inPosition() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                setPathState(AutoStates.GO_CLEAR);
                break;
            case GO_CLEAR:
                robot.blob.setTargetPosition(constants.clear);
                if (!robot.blob.inPosition() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime() && !robot.blob.isStuck()) break;
                setPathState(AutoStates.GO_TO_SCORE1);
                break;
            case GO_TO_SCORE1:
                if (!robot.blob.inPosition() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                robot.outtake.specificValues(constants.scorePose);
                robot.blob.setTargetPosition(constants.scorePose);
                setPathState(AutoStates.WAIT_SCORE_1);
                break;
            case WAIT_SCORE_1:
                if (!robot.blob.inPosition()) break;
                robot.outtake.start_feed_rapid(constants.getLauncherVelocity(), constants.getHoodPosition());
                sleep(constants.getShootingTime(), AutoStates.GO_PICKUP2_INTERMEDIARY);
                break;
            case GO_PICKUP2_INTERMEDIARY:
                robot.outtake.setOuttakeState(Outtake.OuttakeState.IDLE);
                robot.blob.setTargetPosition(constants.pickUpPose2Intermediary);
                if (!robot.blob.inPosition() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                setPathState(AutoStates.GO_PICKUP2);
                break;
            case GO_PICKUP2:
                robot.outtake.setOuttakeState(Outtake.OuttakeState.IDLE);
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.INTAKE);
                robot.blob.setTargetPosition(constants.pickUpPose2);
                setPathState(AutoStates.GO_TO_SCORE2);
                break;
            case GO_TO_SCORE2:
                if (!robot.blob.inPosition() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF);
                robot.outtake.specificValues(constants.scorePose);
                robot.blob.setTargetPosition(constants.scorePose);
                setPathState(AutoStates.WAIT_SCORE2);
                break;
            case WAIT_SCORE2:
                if (!robot.blob.inPosition() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                robot.outtake.start_feed_rapid(constants.getLauncherVelocity(), constants.getHoodPosition());
                sleep(constants.getShootingTime(), AutoStates.GO_PICKUP3_INTERMEDIARY);
                break;
            case GO_PICKUP3_INTERMEDIARY:
                robot.blob.setTargetPosition(constants.pickUpPose3Intermediary);
                robot.outtake.setOuttakeState(Outtake.OuttakeState.IDLE);
                if (!robot.blob.inPosition() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                setPathState(AutoStates.GO_PICKUP3);
                break;
            case GO_PICKUP3:
                robot.outtake.setOuttakeState(Outtake.OuttakeState.IDLE);
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.INTAKE);
                robot.blob.setTargetPosition(constants.pickUpPose3);
                setPathState(AutoStates.GO_TO_SCORE3);
                break;
            case GO_TO_SCORE3:
                if (!robot.blob.inPosition() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                robot.outtake.specificValues(constants.scorePose);
                robot.blob.setTargetPosition(constants.scorePose);
                setPathState(AutoStates.WAIT_SCORE3);
                break;
            case WAIT_SCORE3:
                if (!robot.blob.inPosition()) break;
                robot.outtake.start_feed_rapid(constants.getLauncherVelocity(), constants.getHoodPosition());
                sleep(constants.getShootingTime(), AutoStates.GO_TO_PARK);
                break;
            case GO_TO_PARK:
                robot.outtake.setOuttakeState(Outtake.OuttakeState.IDLE);
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF);
                robot.blob.setTargetPosition(constants.parkPose);
                setPathState(AutoStates.PARK);
                break;
            case PARK:
                if (!robot.blob.inPosition()) break;
                requestOpModeStop();
                break;
            case SLEEP:
                if (System.currentTimeMillis() - startSleep > sleeptime) {
                    setPathState(nextState);
                }
                break;
        }
//        TelemetryUtil.packet.put("x", robot.blob.odo.x);
//        TelemetryUtil.packet.put("y", robot.blob.odo.y);
//        TelemetryUtil.packet.put("heading", robot.blob.odo.heading);
//        TelemetryUtil.sendTelemetry();
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
