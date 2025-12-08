package org.firstinspires.ftc.teamcode.Hardware.Intake;

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

public  class IntakeTransfer implements Module {
    public CachingDcMotorEx motor1,motor2;
    private Robot robot;

    CachingServo intakeServo;
    CachingServo ramp;



    public enum IntakeState {
        OFF,
        INTAKE,
        REVERSE,
        START_TRANSFER,
        SLEEP,
        TRANSFER,
        HOLD
    }

    public enum RampState {
        CLOSE,
        OPEN
    }

    public enum ServoIntakeState{
        LOW,
        HIGH
    }

    public IntakeState intakeState = IntakeState.OFF;
    public RampState rampState =  RampState.CLOSE;
    public ServoIntakeState servoIntakeState =ServoIntakeState.LOW;
    long startSleep = 0;
    double sleeptime = 0;
    IntakeState nextState = IntakeState.OFF;

    public IntakeTransfer(Robot robot, Sensors sensors) {
        this.robot = robot;
        motor1 = new CachingDcMotorEx(robot.hw.get(DcMotorEx.class,"conveyor1"),0);
        motor2 = new CachingDcMotorEx(robot.hw.get(DcMotorEx.class,"conveyor2"),0);

        intakeServo = new CachingServo(robot.hw.get(Servo.class,"intakeTilt"));
        ramp = new CachingServo(robot.hw.get(Servo.class,"ramp"));
        HardwareUtils.unlock(motor1);
        HardwareUtils.unlock(motor2);
        motor1.setCurrentAlert(IntakeConstants.intakeAmpsThreshold, CurrentUnit.AMPS);
        motor1.setDirection(IntakeConstants.isConveyer1Reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(IntakeConstants.isConveyer2Reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

    }

    @Override
    public void update()
    {
        switch (intakeState) {
            case OFF:
                rampState = RampState.CLOSE;
                motor1.setPower(0);
                motor2.setPower(0);
                break;
            case INTAKE:
                rampState = RampState.CLOSE;
                servoIntakeState = ServoIntakeState.LOW;
                motor1.setPower(IntakeConstants.intakePower);
                motor2.setPower(IntakeConstants.intakePower);
                if(motor1.isOverCurrent()){
                    intakeState=IntakeState.OFF;
                }
                break;
            case REVERSE:
                rampState = RampState.CLOSE;
                motor1.setPower(IntakeConstants.reversePower);
                motor2.setPower(IntakeConstants.reversePower);
                break;
            case START_TRANSFER:
                rampState = RampState.OPEN;
                sleep(250,IntakeState.TRANSFER);
                break;
            case TRANSFER:
                motor1.setPower(IntakeConstants.transferPower);
                motor2.setPower(IntakeConstants.transferPower);
                break;
            case SLEEP:
                if(System.currentTimeMillis() - startSleep > sleeptime) {
                    intakeState = nextState;
                }
                break;
        }

        switch (rampState){
            case OPEN:
                ramp.setPosition(IntakeConstants.rampOpen);
                break;
            case CLOSE:
                ramp.setPosition(IntakeConstants.rampClose);
                break;
        }
        switch (servoIntakeState){
            case LOW:
                intakeServo.setPosition(IntakeConstants.intakeServoLow);
                break;
            case HIGH:
                intakeServo.setPosition(IntakeConstants.intakeServoHigh);
                break;
        }

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

    private void sleep(double time,IntakeState nextState) {
        startSleep = System.currentTimeMillis();
        intakeState = nextState;
        sleeptime = time;
        this.nextState = nextState;
    }
}
