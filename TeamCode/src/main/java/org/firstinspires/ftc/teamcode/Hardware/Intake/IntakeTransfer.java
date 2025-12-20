package org.firstinspires.ftc.teamcode.Hardware.Intake;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware.Module;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.Sensors;
import org.firstinspires.ftc.teamcode.Util.Caching.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.Util.Caching.CachingServo;
import org.firstinspires.ftc.teamcode.Util.HardwareUtils;
import org.firstinspires.ftc.teamcode.Util.Wrapper.BinaryDeque;

@Config
public class IntakeTransfer implements Module {
    public CachingDcMotorEx motor1, motor2;
    private Robot robot;

    CachingServo intakeServo;
    CachingServo left;
    CachingServo right;
    CachingServo ramp;

    public static boolean useStall = false;


    public enum IntakeState {
        OFF,
        INTAKE,
        REVERSE,
        START_TRANSFER,
        POWER_FOR_TIME,
        OFF_OPEN,
        SLEEP,
        TRANSFER,
        HOLD
    }

    public enum RampState {
        CLOSE,
        OPEN
    }

    public enum ServoIntakeState {
        LOW,
        INTAKE,
        HIGH
    }

    public enum StallCheck {
        IDLE,
        DETECTED,
        CONFIRMING
    }

    public StallCheck stallCheck = StallCheck.IDLE;

    public IntakeState intakeState = IntakeState.OFF;
    public RampState rampState = RampState.CLOSE;
    public ServoIntakeState servoIntakeState = ServoIntakeState.INTAKE;
    long startSleep = 0;
    double sleeptime = 0;
    IntakeState nextState = IntakeState.OFF;

    BinaryDeque deq = new BinaryDeque();

    public IntakeTransfer(Robot robot, Sensors sensors) {
        this.robot = robot;
        motor1 = new CachingDcMotorEx(robot.hw.get(DcMotorEx.class, "conveyor1"), 0);
        motor2 = new CachingDcMotorEx(robot.hw.get(DcMotorEx.class, "conveyor2"), 0);

        intakeServo = new CachingServo(robot.hw.get(Servo.class, "intakeTilt"));
        ramp = new CachingServo(robot.hw.get(Servo.class, "ramp"));
        left = new CachingServo(robot.hw.get(Servo.class, "left"));
        right = new CachingServo(robot.hw.get(Servo.class, "right"));
        HardwareUtils.unlock(motor1);
        HardwareUtils.unlock(motor2);
        motor1.setCurrentAlert(IntakeConstants.intakeAmpsThreshold, CurrentUnit.AMPS);
        motor1.setDirection(IntakeConstants.isConveyer1Reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(IntakeConstants.isConveyer2Reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

    }

    public double power_time = 0.5;
    public double time_power = 500;
    public double startStallCheckTime = 0;
    public double stalCheckDuration = 150;

    @Override
    public void update() {
        switch (intakeState) {
            case OFF:
                rampState = RampState.CLOSE;
                servoIntakeState = ServoIntakeState.INTAKE;
                motor1.setPower(0);
                motor2.setPower(0);
                break;
            case OFF_OPEN:
                rampState = RampState.OPEN;
                motor1.setPower(0);
                motor2.setPower(0);
                break;
            case INTAKE:
                rampState = RampState.CLOSE;
                servoIntakeState = ServoIntakeState.INTAKE;
                motor1.setPower(IntakeConstants.intakePowerConveyer);
                motor2.setPower(IntakeConstants.intakeShushi);

                break;
            case REVERSE:
                rampState = RampState.CLOSE;
                motor1.setPower(-IntakeConstants.reversePower);
                motor2.setPower(-IntakeConstants.reversePower);
                break;
            case START_TRANSFER:
                rampState = RampState.OPEN;
                sleep(250, IntakeState.TRANSFER);
                break;
            case POWER_FOR_TIME:
                motor1.setPower(power_time);
                motor2.setPower(power_time);
                sleep(time_power, IntakeState.OFF_OPEN);
                break;
            case TRANSFER:
                servoIntakeState = ServoIntakeState.LOW;
                motor1.setPower(IntakeConstants.transferPowerConveyer);
                motor2.setPower(IntakeConstants.transferPowerSushi);
                break;
            case SLEEP:
                Log.w("Debug shoot precise", " " + (System.currentTimeMillis() - startSleep));

                if (System.currentTimeMillis() - startSleep > sleeptime) {
                    intakeState = nextState;
                }
                break;
        }

        switch (rampState) {
            case OPEN:
                ramp.setPosition(IntakeConstants.rampOpen);
                break;
            case CLOSE:
                ramp.setPosition(IntakeConstants.rampClose);
                break;
        }
        switch (servoIntakeState) {
            case LOW:
                intakeServo.setPosition(IntakeConstants.intakeServoLow);
                break;
            case INTAKE:
                intakeServo.setPosition(IntakeConstants.intakeServoIntake);
                break;
            case HIGH:
                intakeServo.setPosition(IntakeConstants.intakeServoHigh);
                break;
        }


        if (useStall) {
            switch (stallCheck) {
                case IDLE:
                    if (intakeState == IntakeState.INTAKE) {
                        stallCheck = StallCheck.DETECTED;
                    }
                    break;
                case DETECTED:
                    if (intakeState != IntakeState.INTAKE) {
                        stallCheck = StallCheck.IDLE;
                        break;
                    }
                    if (motor1.isOverCurrent()) {
                        startStallCheckTime = System.currentTimeMillis();
                        stallCheck = StallCheck.CONFIRMING;
                    }
                    break;
                case CONFIRMING:
                    if (intakeState != IntakeState.INTAKE) {
                        stallCheck = StallCheck.IDLE;
                        break;
                    }
                    if (!motor1.isOverCurrent()) {
                        stallCheck = StallCheck.DETECTED;
                        break;
                    }
                    if (System.currentTimeMillis() - startStallCheckTime > stalCheckDuration) {
                        Log.w("IntakeTransfer", "Stall detected, stopping intake");
                        setIntakeState(IntakeState.OFF);
                        stallCheck = StallCheck.IDLE;
                    }
                    break;

            }
        }

        left.setPosition(IntakeConstants.leftTransfer);
        right.setPosition(IntakeConstants.rightTransfer);
    }

    public void setIntakeState(IntakeState intakeState) {
        this.intakeState = intakeState;
    }

    public void setRampState(RampState rampState) {
        this.rampState = rampState;
    }

    public void setServoIntakeState(ServoIntakeState servoIntakeState) {
        this.servoIntakeState = servoIntakeState;
    }

    public void increaseIntakeServo(double delta) {
        
    }
    public void setPowerForTime(double power, double time) {
        this.power_time = power;
        this.time_power = time;
        intakeState = IntakeState.POWER_FOR_TIME;
    }

    private void sleep(double time, IntakeState nextState) {
        startSleep = System.currentTimeMillis();
        intakeState = IntakeState.SLEEP;
        sleeptime = time;
        this.nextState = nextState;
    }
}