package org.firstinspires.ftc.teamcode.OpMode.Auto.Close.Close15Blob;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Intake.IntakeTransfer;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Turret;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.OpMode.Auto.Close.Close12Blob.Close12Blob;
import org.firstinspires.ftc.teamcode.Util.Globals.Phase;
import org.firstinspires.ftc.teamcode.Util.Info;
import org.firstinspires.ftc.teamcode.Util.Wrapper.TelemetryUtil;


// DEPRICATED DO NOT USE



public class CloseBlob extends OpMode {
    Robot robot;

    Timer pathTimer;
    int gateCycleCounter = 0;

    ElapsedTime timerAuto = null;
    public enum AutoStates {
        IDLE,
        GO_TO_SCORE_FROM_START,
        WAIT_SCORE_PRELOAD,
        GO_PICKUP2,
        GO_PICKUP2_2,
        GO_TO_SCORE2,
        WAIT_SCORE2,
        GO_GATE_PICKUP,
        GATE_PATH_DONE,
        WAIT_GATE_PICKUP,
        GO_SCORE_GATE_PICKUP,
        WAIT_SCORE_GATE_PICKUP,
        GO_PICKUP1,
        GO_CLEAR,
        GO_CLEAR_INTER,
        GO_TO_SCORE1,
        WAIT_SCORE_1,
        GO_TO_PARK,
        PARK,
        SLEEP
    }

    public AutoStates autoStates = AutoStates.IDLE;
    CloseBlobConstants constants;

    @Override
    public void init() {
        Info.phase = Phase.AUTONOMOUS;
        Info.useBlob = true;
        robot = new Robot(this);


        constants = new CloseBlobConstants();
        constants.buildPaths();

        
        robot.outtake.turret.turretState = Turret.TurretState.TRACKING;
        robot.outtake.launcher.autoAimOn(true);


    }

    @Override
    public void start() {
        robot.blob.odo.setPose(constants.startPose);
        robot.blob.odo.update();
        pathTimer = new Timer();
        gateCycleCounter = 0;
        setPathState(AutoStates.GO_TO_SCORE_FROM_START);
        timerAuto = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timerAuto.startTime();
    }

    @Override
    public void loop() {
        switch (autoStates) {
            case IDLE:
                break;
            case GO_TO_SCORE_FROM_START:
                //robot.drive.followPath(constants.scorePreload, true);
                robot.blob.setTargetPosition(constants.scorePose);
                robot.outtake.specificValues(constants.scorePose);
                setPathState(AutoStates.WAIT_SCORE_PRELOAD);
                break;
            case WAIT_SCORE_PRELOAD:
                if (!robot.blob.inPosition() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                robot.outtake.start_feed_rapid(constants.getLauncherVelocity(), constants.getHoodPosition());
                sleep(constants.getShootingTime(), AutoStates.GO_PICKUP1);
                break;
            case GO_PICKUP2:

                //robot.drive.followPath(constants.grabPickup2, constants.getMaxPower(), true);
                robot.blob.setTargetPosition(constants.pickUpPose2);
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.INTAKE);
                if (!robot.blob.inPosition() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                setPathState(AutoStates.GO_PICKUP2_2);
                break;
            case GO_PICKUP2_2:
                robot.blob.setTargetPosition(constants.pickUpPose2_2);
                if (!robot.blob.inPosition() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                setPathState(AutoStates.GO_CLEAR_INTER);
                break;
            case GO_TO_SCORE2:

                if (!robot.blob.inPosition() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                robot.outtake.specificValues(constants.scorePose);
                //robot.drive.followPath(constants.scorePickup2, true);
                robot.blob.setTargetPosition(constants.scorePose);
                setPathState(AutoStates.WAIT_SCORE2);
                break;
            case WAIT_SCORE2:
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF);
                if (!robot.blob.inPosition() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                robot.outtake.start_feed_rapid(constants.getLauncherVelocity(), constants.getHoodPosition());
                sleep(constants.getShootingTime(), AutoStates.GO_GATE_PICKUP);
                break;
            case GO_GATE_PICKUP:
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF);
                robot.outtake.setOuttakeState(Outtake.OuttakeState.IDLE);
                //robot.drive.followPath(constants.grabGatePickup, constants.getMaxPower(), true);
                robot.blob.setTargetPosition(constants.gatePickupPose);
                setPathState(AutoStates.GATE_PATH_DONE);
                break;
            case GATE_PATH_DONE:
                if(robot.blob.progress > 0.5) robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.INTAKE);
                if (!robot.blob.inPosition() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                else {
                    robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.INTAKE);
                    setPathState(AutoStates.WAIT_GATE_PICKUP);
                }
                break;
            case WAIT_GATE_PICKUP:
                if (robot.intakeTransfer.intakeState == IntakeTransfer.IntakeState.OFF || pathTimer.getElapsedTime() > constants.getFailSafePickupTime()) {
                    robot.outtake.specificValues(constants.scorePose);
                    setPathState(AutoStates.GO_SCORE_GATE_PICKUP);
                }
                break;
            case GO_SCORE_GATE_PICKUP:
                if(timerAuto.time() > constants.getParkThreeshold()) {
                    setPathState(AutoStates.GO_TO_PARK);
                    break;
                }
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF);
                //robot.drive.followPath(constants.scoreGatePickup, true);
                robot.blob.setTargetPosition(constants.scorePoseGateInter);
                if(robot.blob.progress > constants.percentage) {
                    setPathState(AutoStates.WAIT_SCORE_GATE_PICKUP);
                    robot.blob.setTargetPosition(constants.scorePose);
                }


                break;
            case WAIT_SCORE_GATE_PICKUP:
                if (!robot.blob.inPosition() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                robot.outtake.start_feed_rapid(constants.getLauncherVelocity(), constants.getHoodPosition());
                gateCycleCounter++;
                if (gateCycleCounter < CloseBlobConstants.gateCycleCount) {
                    sleep(constants.getShootingTime(), AutoStates.GO_GATE_PICKUP);
                } else {
                    sleep(constants.getShootingTime(), AutoStates.GO_TO_PARK);
                }
                break;
            case GO_PICKUP1:
                robot.outtake.setOuttakeState(Outtake.OuttakeState.IDLE);
                //robot.drive.followPath(constants.grabPickUp1, constants.getMaxPower(), true);
                robot.blob.setTargetPosition(constants.pickUpPose);
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.INTAKE);
                setPathState(AutoStates.GO_TO_SCORE1);
                break;
            case GO_CLEAR_INTER:
                robot.blob.setTargetPosition(constants.clearInter);
                if (!robot.blob.inPosition() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                setPathState(CloseBlob.AutoStates.GO_CLEAR);
                break;
            case GO_CLEAR:
                if (!robot.blob.inPosition() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF);
                robot.blob.setTargetPosition(constants.clear);
                setPathState(AutoStates.GO_TO_SCORE2);
                break;
            case GO_TO_SCORE1:
                if (!robot.blob.inPosition() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                robot.outtake.specificValues(constants.scorePose);
                //robot.drive.followPath(constants.scorePickup1, true);
                robot.blob.setTargetPosition(constants.scorePose);
                setPathState(AutoStates.WAIT_SCORE_1);
                break;
            case WAIT_SCORE_1:
                if (!robot.blob.inPosition() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF);
                robot.outtake.start_feed_rapid(constants.getLauncherVelocity(), constants.getHoodPosition());
                sleep(constants.getShootingTime(), AutoStates.GO_PICKUP2);
                break;
            case GO_TO_PARK:
                robot.outtake.setOuttakeState(Outtake.OuttakeState.IDLE);                //robot.drive.followPath(constants.goToPark, true);
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
        TelemetryUtil.packet.put("x", robot.blob.odo.x);
        TelemetryUtil.packet.put("y", robot.blob.odo.y);
        TelemetryUtil.packet.put("heading", robot.blob.odo.heading);
        TelemetryUtil.sendTelemetry();
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
