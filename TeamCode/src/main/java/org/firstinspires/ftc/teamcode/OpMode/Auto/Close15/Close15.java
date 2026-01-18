package org.firstinspires.ftc.teamcode.OpMode.Auto.Close15;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware.Intake.IntakeTransfer;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Turret;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Util.Wrapper.GamePadController;

@Autonomous(name = "Close15")
public class Close15 extends OpMode {
    Robot robot;
    Timer pathTimer;
    GamePadController gg;

    boolean disablePickup1 = false;
    boolean disableClear = false;
    boolean disablePickup2 = false;
    boolean disablePickup3 = false;
    boolean disableClearPickup = false;

    int selectedIndex = 0; // 0: pickup1, 1: clear, 2: pickup2, 3: supercycle, 4: pickup3

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
        GO_CLEAR2,
        GO_CLEAR_PICKUP,
        GO_TO_SCORE_CLEAR_PICKUP,
        WAIT_SCORE_CLEAR_PICKUP,
        GO_PICKUP3,
        CLEAR,
        GO_TO_SCORE3,
        WAIT_SCORE3,
        GO_TO_PARK,
        PARK,
        SLEEP

    }

    public AutoStates autoStates = AutoStates.IDLE;
    CloseConstants15 constants;

    @Override
    public void init() {
        robot = new Robot(this);
        constants = new CloseConstants15();
        constants.buildPaths(robot.drive);
        robot.drive.setPose(constants.startPose);


        robot.outtake.turret.turretState = Turret.TurretState.FIXED_ANGLE;
        robot.outtake.launcher.autoAimOn(true);

        gg = new GamePadController(gamepad1);
    }

    @Override
    public void init_loop() {
        gg.update();

        if (gg.dpadUpOnce()) {
            selectedIndex--;
            if (selectedIndex < 0) selectedIndex = 4;
        }
        if (gg.dpadDownOnce()) {
            selectedIndex++;
            if (selectedIndex > 4) selectedIndex = 0;
        }

        if (gg.aOnce()) {
            switch (selectedIndex) {
                case 0:
                    disablePickup1 = !disablePickup1;
                    break;
                case 1:
                    disableClear = !disableClear;
                    break;
                case 2:
                    disablePickup2 = !disablePickup2;
                    break;
                case 3:
                    disableClearPickup = !disableClearPickup;
                    break;
                case 4:
                    disablePickup3 = !disablePickup3;
                    break;
            }
        }

        telemetry.addLine("Close Auto Config (A = toggle, dpad up/down = move)");
        telemetry.addLine(formatOptionLine(0, "Disable Pickup 1", disablePickup1));
        telemetry.addLine(formatOptionLine(1, "Disable Clear", disableClear));
        telemetry.addLine(formatOptionLine(2, "Disable Pickup 2", disablePickup2));
        telemetry.addLine(formatOptionLine(3, "Disable Supercycle", disableClearPickup));
        telemetry.addLine(formatOptionLine(4, "Disable Pickup 3", disablePickup3));
        telemetry.update();
    }

    private String formatOptionLine(int index, String label, boolean enabled) {
        char cursorOrSpace = (selectedIndex == index) ? '*' : ' ';
        char tick = enabled ? 'X' : ' ';
        return String.format("%c [%c] %s", cursorOrSpace, tick, label);
    }

    @Override
    public void start() {
        pathTimer = new Timer();
        setPathState(AutoStates.GO_TO_SCORE_FROM_START);
    }

    @Override
    public void loop() {
        switch (autoStates) {
            case IDLE:
                break;
            case GO_TO_SCORE_FROM_START:
                robot.drive.followPath(constants.scorePreload,true);
                robot.outtake.turret.setPosFixed(constants.getTurretPosition());
                setPathState(AutoStates.WAIT_SCORE_PRELOAD);
                break;
            case WAIT_SCORE_PRELOAD:
                if (robot.drive.isBusy() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                robot.outtake.start_feed_rapid(constants.getLauncherVelocity(), constants.getHoodPosition());
                // After shooting preload, determine next state based on disable flags
                sleep(constants.getShootingTime(), getNextStateAfterPreload());
                break;
            case GO_PICKUP1:
                robot.outtake.setOuttakeState(Outtake.OuttakeState.IDLE);
                robot.drive.followPath(constants.grabPickUp1, constants.getMaxPower(),true);
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.INTAKE);
                // After pickup1, check if clear is enabled
                if (disableClear) {
                    setPathState(AutoStates.GO_TO_SCORE1);
                } else {
                    setPathState(AutoStates.CLEAR);
                }
                break;
            case CLEAR:
                // CLEAR happens between GO_PICKUP1 and GO_TO_SCORE1
                // If we're here, we need to score afterward (skip logic is in getNextStateAfterPreload)
                if (robot.drive.isBusy() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF);
                robot.drive.followPath(constants.goClear, constants.getMaxClearPower(), false);
                // After clear, go to score1 to shoot what we collected
                setPathState(AutoStates.GO_TO_SCORE1);
                break;
            case GO_TO_SCORE1:
                if (robot.drive.isBusy() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                robot.drive.followPath(constants.scorePickup1);
                setPathState(AutoStates.WAIT_SCORE_1);
                break;
            case WAIT_SCORE_1:
                if (robot.drive.isBusy()) break;
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF);
                robot.outtake.start_feed_rapid(constants.getLauncherVelocity(), constants.getHoodPosition());
                // After shooting pickup1, determine next state based on disable flags
                sleep(constants.getShootingTime(), getNextStateAfterPickup1());
                break;
            case GO_PICKUP2:
                robot.outtake.setOuttakeState(Outtake.OuttakeState.IDLE);
                robot.drive.followPath(constants.grabPickup2, constants.getMaxPower(), true);
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.INTAKE);
                setPathState(AutoStates.GO_TO_SCORE2);
                break;
            case GO_TO_SCORE2:
                if (robot.drive.isBusy() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                robot.drive.followPath(constants.scorePickup2,false);
                setPathState(AutoStates.WAIT_SCORE2);
                break;
            case WAIT_SCORE2:
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF);
                if (robot.drive.isBusy()) break;
                robot.outtake.start_feed_rapid(constants.getLauncherVelocity(), constants.getHoodPosition());
                // After shooting pickup2, determine next state based on disable flags
                sleep(constants.getShootingTime(), getNextStateAfterPickup2());
                break;
            case GO_CLEAR2:
                robot.outtake.setOuttakeState(Outtake.OuttakeState.IDLE);
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF);
                robot.drive.followPath(constants.goClear2, constants.getMaxClearPower(), true);
                setPathState(AutoStates.GO_CLEAR_PICKUP);
                break;
            case GO_CLEAR_PICKUP:
                robot.outtake.setOuttakeState(Outtake.OuttakeState.IDLE);
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.INTAKE);
                robot.drive.followPath(constants.clearPickup, constants.getMaxPower(), true);
                setPathState(AutoStates.GO_TO_SCORE_CLEAR_PICKUP);
                break;
            case GO_TO_SCORE_CLEAR_PICKUP:
                if (robot.drive.isBusy() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                robot.drive.followPath(constants.scoreClearPickup, true);
                setPathState(AutoStates.WAIT_SCORE_CLEAR_PICKUP);
                break;
            case WAIT_SCORE_CLEAR_PICKUP:
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF);
                if (robot.drive.isBusy()) break;
                robot.outtake.start_feed_rapid(constants.getLauncherVelocity(), constants.getHoodPosition());
                // After shooting clearPickup, determine next state based on disable flags
                sleep(constants.getShootingTime(), getNextStateAfterClearPickup());
                break;
            case GO_PICKUP3:
                robot.outtake.setOuttakeState(Outtake.OuttakeState.IDLE);
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
                robot.outtake.setOuttakeState(Outtake.OuttakeState.IDLE);
                robot.drive.followPath(constants.goToPark);
                setPathState(AutoStates.PARK);
                break;
            case PARK:
                if(robot.drive.isBusy()) break;
                robot.outtake.setOuttakeState(Outtake.OuttakeState.IDLE);

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

    // Helper methods to determine next state based on disable flags

    // After preload: check pickup1 -> (clear is part of pickup1 cycle) -> pickup2 -> clearPickup -> pickup3 -> park
    private AutoStates getNextStateAfterPreload() {
        // If pickup1 enabled, go to pickup1 (which will check clear internally)
        if (!disablePickup1) return AutoStates.GO_PICKUP1;
        // If pickup1 disabled but clear enabled, go directly to clear (then to score1)
        if (!disableClear) return AutoStates.CLEAR;
        // Both pickup1 and clear disabled, check pickup2
        if (!disablePickup2) return AutoStates.GO_PICKUP2;
        if (!disableClearPickup) return AutoStates.GO_CLEAR2;
        if (!disablePickup3) return AutoStates.GO_PICKUP3;
        return AutoStates.GO_TO_PARK;
    }

    // After pickup1 shot: check pickup2 -> clearPickup -> pickup3 -> park
    // (clear is already handled before GO_TO_SCORE1)
    private AutoStates getNextStateAfterPickup1() {
        if (!disablePickup2) return AutoStates.GO_PICKUP2;
        if (!disableClearPickup) return AutoStates.GO_CLEAR2;
        if (!disablePickup3) return AutoStates.GO_PICKUP3;
        return AutoStates.GO_TO_PARK;
    }

    // After pickup2 shot: check clearPickup -> pickup3 -> park
    private AutoStates getNextStateAfterPickup2() {
        if (!disableClearPickup) return AutoStates.GO_CLEAR2;
        if (!disablePickup3) return AutoStates.GO_PICKUP3;
        return AutoStates.GO_TO_PARK;
    }

    // After clearPickup shot: check pickup3 -> park
    private AutoStates getNextStateAfterClearPickup() {
        if (!disablePickup3) return AutoStates.GO_PICKUP3;
        return AutoStates.GO_TO_PARK;
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
