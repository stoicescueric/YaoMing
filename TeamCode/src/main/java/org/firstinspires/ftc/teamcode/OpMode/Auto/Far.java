package org.firstinspires.ftc.teamcode.OpMode.Auto;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware.Intake.IntakeTransfer;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.OuttakePositions;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Turret;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Util.Wrapper.GamePadController;

@Autonomous(name = "Far")
public class Far extends OpMode {
    Robot robot;
    Timer pathTimer;
    GamePadController gg;

    // Simple config: allow disabling pickup1 or pickup2
    boolean disablePickup1 = false;
    boolean disablePickup2 = false;

    int selectedIndex = 0; // 0: pickup1, 1: pickup2

    public enum AutoStates {
        IDLE,
        GO_TO_SCORE_FROM_START,
        WAIT_SCORE_PRELOAD,
        GO_PICKUP1,
        GO_TO_SCORE1,
        WAIT_SCORE_1,
        GO_PICKUP2,
        GO_TO_SCORE2,
        WAIT_SCORE2,
        GO_TO_PARK,
        PARK,
        SLEEP
    }

    public AutoStates autoStates = AutoStates.IDLE;
    FarConstants constants;

    @Override
    public void init() {
        robot = new Robot(this);
        constants = new FarConstants();
        constants.buildPaths(robot.drive);
        robot.drive.setPose(constants.startPose);
        robot.outtake.turret.turretState = Turret.TurretState.TRACKING;
        robot.outtake.launcher.autoAimOn(true);

        gg = new GamePadController(gamepad1);
    }

    @Override
    public void init_loop() {
        gg.update();

        if (gg.dpadUpOnce()) {
            selectedIndex--;
            if (selectedIndex < 0) selectedIndex = 1;
        }
        if (gg.dpadDownOnce()) {
            selectedIndex++;
            if (selectedIndex > 1) selectedIndex = 0;
        }

        if (gg.aOnce()) {
            switch (selectedIndex) {
                case 0:
                    disablePickup1 = !disablePickup1;
                    break;
                case 1:
                    disablePickup2 = !disablePickup2;
                    break;
            }
        }

        telemetry.addLine("Far Auto (Close-style) Config (A = toggle, dpad up/down = move)");
        telemetry.addLine(formatOptionLine(0, "Disable Pickup 1", disablePickup1));
        telemetry.addLine(formatOptionLine(1, "Disable Pickup 2", disablePickup2));
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
                robot.drive.followPath(constants.scorePreload, true);
                robot.outtake.turret.turretState = Turret.TurretState.TRACKING;
                robot.outtake.launcher.autoAimOn(true);
                setPathState(AutoStates.WAIT_SCORE_PRELOAD);
                break;

            case WAIT_SCORE_PRELOAD:
                if (robot.drive.isBusy() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                robot.outtake.start_feed_precise(OuttakePositions.farLaunchVelocity, OuttakePositions.farLaunchTilt);
                if (disablePickup1) {
                    if (disablePickup2) {
                        sleep(constants.getShootingTime(), AutoStates.GO_TO_PARK);
                    } else {
                        sleep(constants.getShootingTime(), AutoStates.GO_PICKUP2);
                    }
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
                robot.drive.followPath(constants.scorePickup1);
                setPathState(AutoStates.WAIT_SCORE_1);
                break;

            case WAIT_SCORE_1:
                if (robot.drive.isBusy()) break;
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF);
                robot.outtake.start_feed_precise(OuttakePositions.farLaunchVelocity, OuttakePositions.farLaunchTilt);
                if (disablePickup2) {
                    sleep(constants.getShootingTime(), AutoStates.GO_TO_PARK);
                } else {
                    sleep(constants.getShootingTime(), AutoStates.GO_PICKUP2);
                }
                break;

            case GO_PICKUP2:
                robot.outtake.setOuttakeState(Outtake.OuttakeState.IDLE);
                robot.drive.followPath(constants.grabPickup2, constants.getMaxPower(), true);
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.INTAKE);
                setPathState(AutoStates.GO_TO_SCORE2);
                break;

            case GO_TO_SCORE2:
                if (robot.drive.isBusy() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                robot.drive.followPath(constants.scorePickup2, false);
                setPathState(AutoStates.WAIT_SCORE2);
                break;

            case WAIT_SCORE2:
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF);
                if (robot.drive.isBusy()) break;
                robot.outtake.start_feed_precise(OuttakePositions.farLaunchVelocity, OuttakePositions.farLaunchTilt);
                sleep(constants.getShootingTime(), AutoStates.GO_TO_PARK);
                break;

            case GO_TO_PARK:
                robot.outtake.setOuttakeState(Outtake.OuttakeState.IDLE);
                robot.drive.followPath(constants.goToPark);
                setPathState(AutoStates.PARK);
                break;

            case PARK:
                if (robot.drive.isBusy()) break;
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
