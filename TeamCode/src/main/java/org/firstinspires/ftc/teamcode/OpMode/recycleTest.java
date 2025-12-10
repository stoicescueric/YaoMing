package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.Intake.IntakeTransfer;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Launcher;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Util.Wrapper.GamePadController;

@Config
@TeleOp(name = "Recycle")
public class recycleTest extends LinearOpMode {

    Robot robot;
    GamePadController gg;
    public enum StartRecycle {
        IDLE,
        WAIT_SHOOTER,
        TRANSFER,
        DELAY,
        FINISH
    }

    public enum CapacState {
        CLOSED,
    }
    public StartRecycle recycle = StartRecycle.IDLE;
    public static double recylce_vel = 500;
    public static double recycle_tilt = 0.9;
    public static double kicker_close = 0.1;
    public static double kicker_open = 0.9;
    public static double finish_time = 0;
    public static double finish_delay = 500;
    Servo servo;
    public static double delay = 1000;
    public static double delay_starting = 200;
    public static double ramp_delay = 0;
    public static double ramp_delay_starting = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        gg = new GamePadController(gamepad1);
        robot = new Robot(this);

        servo = hardwareMap.get(Servo.class,"Kicker");

        waitForStart();
        robot.outtake.outtakeState = Outtake.OuttakeState.OFF;
        while (opModeIsActive()) {
            gg.update();
            if(gg.aOnce()) {
                switch (robot.intakeTransfer.servoIntakeState) {
                    case LOW:
                        robot.intakeTransfer.servoIntakeState = IntakeTransfer.ServoIntakeState.HIGH;
                        break;
                    case HIGH:
                        robot.intakeTransfer.servoIntakeState = IntakeTransfer.ServoIntakeState.LOW;
                        break;
                }
            }
            if(gg.yOnce()) {
                recycle = StartRecycle.WAIT_SHOOTER;
            }
            switch (recycle) {
                case IDLE:
                    servo.setPosition(kicker_close);
                    break;
                case WAIT_SHOOTER:
                    robot.outtake.launcher.setTarget(recylce_vel,recycle_tilt);
                    if(robot.outtake.launcher.isReady()) {
                        robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.START_TRANSFER);
                        recycle = StartRecycle.TRANSFER;
                    }
                    break;
                case TRANSFER:
                    if(robot.intakeTransfer.intakeState == IntakeTransfer.IntakeState.TRANSFER) {
                        servo.setPosition(kicker_open);
                        delay_starting = System.currentTimeMillis();
                        recycle = StartRecycle.DELAY;
                    }
                    break;
                case DELAY:
                    if(System.currentTimeMillis() - delay_starting > ramp_delay) {
                        robot.intakeTransfer.rampState = IntakeTransfer.RampState.CLOSE;
                    }
                    if(System.currentTimeMillis() - delay_starting > delay) {
                        robot.outtake.launcher.launcherState = Launcher.LauncherState.OFF;
                        servo.setPosition(kicker_close);
                        recycle = StartRecycle.FINISH;
                        finish_time = System.currentTimeMillis();
                    }
                    break;
                case FINISH:
                    if(System.currentTimeMillis() - finish_time > finish_delay) {
                        robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF);
                        recycle = StartRecycle.IDLE;
                    }
                    break;
            }
            telemetry.addData("case",recycle);
            telemetry.update();
            robot.update();

        }
    }
}