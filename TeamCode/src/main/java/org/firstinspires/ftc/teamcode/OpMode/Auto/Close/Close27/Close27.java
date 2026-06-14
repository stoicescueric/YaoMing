package org.firstinspires.ftc.teamcode.OpMode.Auto.Close.Close27;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Intake.IntakeTransfer;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Launcher;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Turret;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.OpMode.Auto.Close.Close18Playoff.CloseConstants18Playoff;
import org.firstinspires.ftc.teamcode.Util.Globals.Alliance;
import org.firstinspires.ftc.teamcode.Util.Globals.Phase;
import org.firstinspires.ftc.teamcode.Util.Info;
import org.firstinspires.ftc.teamcode.Util.Wrapper.TelemetryUtil;

public class Close27 extends OpMode {
    Robot robot;

    Timer pathTimer;
    int gateCycleCounter = 0;
    int gateCycleCount = 5;
    ElapsedTime timerAuto = null;
    public enum AutoStates {
        IDLE,
        GO_TO_SCORE_FROM_START,
        WAIT_SCORE_PRELOAD,
        GO_PICKUP2,
        GO_PICKUP2_2,
        GO_TO_SCORE2,
        WAIT_SCORE2,
        GO_PICKUP3,
        GO_PICKUP3_2,
        GO_TO_SCORE3,
        WAIT_SCORE3,
        GO_GATE_PICKUP,
        GATE_PATH_DONE,
        WAIT_GATE_PICKUP,
        GO_SCORE_GATE_PICKUP,
        WAIT_SCORE_GATE_PICKUP,
        GO_PICKUP1,
        GO_PICKUP1_0,
        GO_CLEAR,
        GO_CLEAR_INTER,
        CYCLE_SOTM,
        SCORE2_SOTM,
        GO_TO_SCORE1,
        WAIT_SCORE_1,
        GO_TO_PARK,
        PARK,
        SLEEP,
        ROTATE_PRELOAD,
    }

    public AutoStates autoStates = AutoStates.IDLE;
    public AutoStates prevAutoStates = AutoStates.IDLE;
    CloseConstants18Playoff constants;

    @Override
    public void init() {
        Info.phase = Phase.AUTONOMOUS;
        Info.useBlob = true;
        robot = new Robot(this);


        constants = new CloseConstants18Playoff();
        constants.buildPaths();


        robot.outtake.launcher.autoAimOn(true);
        robot.outtake.outtakeState = Outtake.OuttakeState.IDLE;
        robot.sensors.setPoseAlign(true);


    }

    @Override
    public void start() {
        robot.sensors.setPoseAlign(true);
        robot.blob.odo.setPose(constants.startPose);
        robot.blob.odo.update();
        pathTimer = new Timer();
        gateCycleCounter = 0;
        setPathState(AutoStates.GO_TO_SCORE_FROM_START);
        timerAuto = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timerAuto.startTime();

    }

    public boolean failSafeGate = false;
    boolean pickup2 = false,go_pickup2 = false,go_clear_intake = false;
    @Override
    public void loop() {
        telemetry.addData("auto state",autoStates);
        telemetry.addData("Drive inPos",robot.blob.inPosition());
        telemetry.addData("outtake state",robot.outtake.outtakeState);
        telemetry.addData("intake state",robot.intakeTransfer.intakeState);
        telemetry.addData("launcher state",robot.outtake.launcher.launcherState);
        telemetry.update();
        switch (autoStates) {
            case IDLE:
                break;
            case GO_TO_SCORE_FROM_START:
                //robot.drive.followPath(constants.scorePreload, true);
                robot.blob.setTargetPosition(constants.preload);
                robot.outtake.specificValues(constants.preload);
                robot.outtake.outtakeState = Outtake.OuttakeState.IDLE;
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF_OPEN);
//                robot.outtake.turret.setPosFixed(constants.getTurretPosition());
                robot.outtake.turret.turretState = Turret.TurretState.TRACKING;
                setPathState(AutoStates.ROTATE_PRELOAD);
                break;
            case ROTATE_PRELOAD:
                if (!robot.blob.inPosition(1.6, 1.6, 0.12) && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                setPathState(AutoStates.WAIT_SCORE_PRELOAD);
                break;
            case WAIT_SCORE_PRELOAD:
                robot.outtake.turret.turretState = Turret.TurretState.TRACKING;
                robot.blob.setTargetPosition(constants.rotatePreload);
                if (!robot.blob.inPosition(3.8, 3.8, 0.15) && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                robot.outtake.start_feed_rapid(constants.getLauncherVelocity(), constants.getHoodPosition());
                sleep(constants.getShootingTime(), AutoStates.GO_PICKUP1_0, true);
                break;
            case GO_PICKUP2:
                robot.outtake.outtakeState = Outtake.OuttakeState.IDLE;
                robot.intakeTransfer.setBlockerState(IntakeTransfer.BlockerState.CLOSE);
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.INTAKE);
                //robot.drive.followPath(constants.grabPickup2, constants.getMaxPower(), true);
                if (!pickup2) {

                    robot.blob.setTargetPosition(constants.pickUpPose2);
                    pickup2 = true;
                }
                setPathState(AutoStates.GO_PICKUP2_2);
                break;
            case GO_PICKUP1_0:
                robot.blob.setTargetPosition(constants.pickUpPose1_Inter);
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.INTAKE);
                if (!robot.blob.getOverPercentage(constants.getPickUp1Percentage()) && pathTimer.getElapsedTime() < constants.getFailSafeDtTime())
                    break;
                setPathState(AutoStates.GO_PICKUP1);
                break;
            case GO_PICKUP1:
                if (timerAuto.milliseconds() > constants.getFailSafePark()) {
                    setPathState(AutoStates.GO_TO_PARK);
                    break;
                }
                robot.outtake.setOuttakeState(Outtake.OuttakeState.IDLE);
                robot.outtake.specificValues(constants.scorePose);
                //robot.drive.followPath(constants.grabPickUp1, constants.getMaxPower(), true);
                robot.blob.setTargetPosition(constants.pickUpPose);
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.INTAKE);
                setPathState(AutoStates.GO_CLEAR);
                break;

            case GO_TO_SCORE1:
                if (!robot.blob.inPosition(1.3,1.3,0.11) && pathTimer.getElapsedTime() < constants.getFailSafeDtTime())
                    break;
                //robot.drive.followPath(constants.scorePickup1, true);
                robot.blob.maxPower = 1;
                robot.blob.setTargetPosition(constants.scoreS1Pose);
                setPathState(AutoStates.WAIT_SCORE_1);
                break;
            case WAIT_SCORE_1:
                if (!robot.blob.inPosition(1.6, 1.6, 0.12) && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF_OPEN);
                robot.outtake.start_feed_rapid(constants.getLauncherVelocity(), constants.getHoodPosition());
                sleep(constants.getShootingTime(), AutoStates.GO_PICKUP2, true);
                break;
            case GO_PICKUP2_2:
                robot.outtake.outtakeState = Outtake.OuttakeState.IDLE;
                robot.outtake.turret.turretState = Turret.TurretState.TRACKING;
                if(!robot.blob.inPosition(2.5,2.7,0.12) && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break; //TEST
                if(!go_pickup2) {
                    robot.blob.maxPower = 1;
                    robot.blob.setTargetPosition(constants.pickUpPose2_2);
                    go_pickup2 = true;
                    break;
                }
                if (!robot.blob.inPosition(1.6,1.6,0.12) && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
//                setPathState(AutoStates.GO_CLEAR_INTER);
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.INTAKE);

                setPathState(AutoStates.GO_TO_SCORE2);
                break;
            case GO_CLEAR_INTER:
                if(!go_clear_intake) {
                    robot.blob.setTargetPosition(constants.clearInter);
                    go_clear_intake = true;
                }
                if (!robot.blob.inPosition() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                setPathState(AutoStates.GO_CLEAR);
                break;
            case GO_CLEAR:
                if (!robot.blob.inPosition() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                robot.blob.setTargetPosition(constants.clear);
                setPathState(AutoStates.GO_TO_SCORE1);
                break;
            case GO_TO_SCORE2:
                //robot.drive.followPath(constants.scorePickup2, true);
                robot.blob.setTargetPosition(constants.scorePose);
                setPathState(AutoStates.WAIT_SCORE2);
                break;
            case WAIT_SCORE2:
                if(robot.blob.progress > constants.getShootingPercentage()) {
                    robot.outtake.launcher.updateOffssetHood = true;
                }
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF_OPEN);
                if (!robot.blob.inPosition(1.3,1.3,0.11) && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                robot.outtake.start_feed_rapid(constants.getLauncherVelocity(), constants.getHoodPosition());
                sleep(constants.getShootingTimeSOTM(), AutoStates.SCORE2_SOTM,true);
                break;
            case SCORE2_SOTM:
                robot.blob.setTargetPosition(constants.gatePickupPose, constants.getHeadingThreeshold());
                sleep(constants.getShootingTimeSOTMdelay(), AutoStates.GO_GATE_PICKUP,true);
                break;
            case GO_GATE_PICKUP:
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF);
                robot.outtake.setOuttakeState(Outtake.OuttakeState.IDLE);
                robot.outtake.specificValues(constants.scorePose);

                //robot.drive.followPath(constants.grabGatePickup, constants.getMaxPower(), true);
                robot.blob.setTargetPosition(constants.gatePickupPose, constants.getHeadingThreeshold());
                setPathState(AutoStates.GATE_PATH_DONE);
                break;
            case GATE_PATH_DONE:
                if(robot.blob.progress > 0.5) {
                    robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.INTAKE);
                    robot.blob.maxPower = 0.7;
                }
                if ((!robot.blob.inPosition() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime())) break; ///TEST
                else {
                    robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.INTAKE);
                    failSafeGate = false;
                    setPathState(AutoStates.WAIT_GATE_PICKUP);
                }
                break;
            case WAIT_GATE_PICKUP:
                if ((robot.intakeTransfer.intakeState == IntakeTransfer.IntakeState.OFF || robot.intakeTransfer.intakeState == IntakeTransfer.IntakeState.PRE_OFF_OPEN || robot.intakeTransfer.intakeState == IntakeTransfer.IntakeState.OFF_OPEN) || pathTimer.getElapsedTime() > constants.getFailSafePickupTime()) {
                    setPathState(AutoStates.GO_SCORE_GATE_PICKUP);
                }
                break;
            case GO_SCORE_GATE_PICKUP:
//                if(timerAuto.time() > constants.getParkThreeshold()) {
//                    setPathState(AutoStates.GO_TO_PARK);
//                    break;
//                }
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF_OPEN);
                //robot.drive.followPath(constants.scoreGatePickup, true);
                robot.blob.maxPower = 1;
                robot.blob.setTargetPosition(constants.scorePoseGateInter);
                if(robot.blob.progress > constants.percentage) {
                    setPathState(AutoStates.WAIT_SCORE_GATE_PICKUP);
                    if (gateCycleCounter < gateCycleCount) {
                        robot.blob.setTargetPosition(constants.scorePose);
                    } else {
                        robot.blob.setTargetPosition(constants.parkPose24);
                    }
                }


                break;
            case WAIT_SCORE_GATE_PICKUP:
                if(robot.blob.progress > constants.getShootingPercentage()) {
                    robot.outtake.launcher.updateOffssetHood = true;
                }

                if (!robot.blob.inPosition(1.3,1.3,0.11) && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF_OPEN);
                robot.outtake.start_feed_rapid(constants.getLauncherVelocity(), constants.getHoodPosition());
                gateCycleCounter++;
                if (gateCycleCounter < gateCycleCount) {
                    sleep(constants.getShootingTimeSOTM(), AutoStates.CYCLE_SOTM,true);
                } else {
                    sleep(constants.getShootingTime(), AutoStates.PARK,true);
                }
                break;
            case CYCLE_SOTM:
                robot.blob.setTargetPosition(constants.gatePickupPose, constants.getHeadingThreeshold());
                sleep(constants.getShootingTimeSOTMdelay(), AutoStates.GO_GATE_PICKUP,true);

                break;
            case GO_TO_PARK:
                robot.outtake.setOuttakeState(Outtake.OuttakeState.IDLE);                //robot.drive.followPath(constants.goToPark, true);
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF);
                robot.blob.setTargetPosition(constants.parkPose);
                setPathState(AutoStates.PARK);
                break;
            case PARK:
                robot.outtake.setOuttakeState(Outtake.OuttakeState.IDLE);                //robot.drive.followPath(constants.goToPark, true);
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF);
                if (!robot.blob.inPosition()) break;

                requestOpModeStop();
                break;
            case SLEEP:
                if (!entered) {
                    if (robot.outtake.launcher.launcherState == Launcher.LauncherState.LAUNCHING) {
                        startSleep = System.currentTimeMillis();
                        entered = true;
                    }
                } else if (System.currentTimeMillis() - startSleep > sleeptime) {
                    setPathState(nextState);
                }
                break;
        }
        TelemetryUtil.packet.put("x", robot.blob.odo.x);
        TelemetryUtil.packet.put("y", robot.blob.odo.y);
        TelemetryUtil.packet.put("heading", robot.blob.odo.heading);
        TelemetryUtil.sendTelemetry();
        prevAutoStates = autoStates;
        robot.update();
    }

    long startSleep = 0;
    double sleeptime = 0;
    AutoStates nextState = AutoStates.IDLE;
    boolean entered = false;
    private void sleep(double time, AutoStates nextState,boolean shooting) {
        entered = !shooting;
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
