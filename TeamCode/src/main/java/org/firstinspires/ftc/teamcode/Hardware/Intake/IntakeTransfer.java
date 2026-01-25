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
    public CachingDcMotorEx intake, conveyor;
    private Robot robot;
    CachingServo powerArm;
    CachingServo blocker;
    CachingServo capac;

    public static boolean useStall = false;
    public boolean stallTriggeredThisLoop = false;


    public enum IntakeState {
        OFF,
        INTAKE,
        REVERSE,
        START_TRANSFER,
        POWER_FOR_TIME,
        OFF_OPEN,
        SLEEP,
        TRANSFER,
        HOLD,
        RECYCLE
    }

    public enum BlockerState {
        CLOSE,
        OPEN
    }

    public enum PowerArmState {
        BLEG,
        INTAKE,
        LOW,
        RECYCLE,
        TRANSFER
    }

    public enum StallCheck {
        IDLE,
        DETECTED,
        CONFIRMING
    }
    public enum ConveyorState {
        OFF,
        ON,
        POWER_FOR_TIME,
        REVERSE
    }
    public enum CapacState{
        BLEG,
        RECYCLE,
        WIGGLEWIGGLEWIGGLE
    }

    public StallCheck stallCheck = StallCheck.IDLE;

    public IntakeState intakeState = IntakeState.OFF;
    public BlockerState blockerState = BlockerState.CLOSE;
    public PowerArmState powerArmState = PowerArmState.INTAKE;
    public ConveyorState conveyorState = ConveyorState.OFF;
    public CapacState capacState = CapacState.BLEG;
    long startSleep = 0;
    double sleeptime = 0;
    public double intakeSensorCounter = 0;
    IntakeState nextState = IntakeState.OFF;

    BinaryDeque deq = new BinaryDeque();

    public IntakeTransfer(Robot robot, Sensors sensors) {
        this.robot = robot;
        intake = new CachingDcMotorEx(robot.hw.get(DcMotorEx.class, "intake"), 0);
        conveyor = new CachingDcMotorEx(robot.hw.get(DcMotorEx.class, "transfer"), 0);

        powerArm = new CachingServo(robot.hw.get(Servo.class, "powerArm"));
        blocker = new CachingServo(robot.hw.get(Servo.class, "blockerMingi"));
        capac = new CachingServo(robot.hw.get(Servo.class, "capac"));
        HardwareUtils.unlock(intake);
        HardwareUtils.unlock(conveyor);
        intake.setCurrentAlert(IntakeConstants.intakeAmpsThreshold, CurrentUnit.AMPS);
        intake.setDirection(IntakeConstants.isIntakeReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        conveyor.setDirection(IntakeConstants.isTransferReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

    }

    public double power_time = 0.5;
    public double time_power = 500;
    public double startStallCheckTime = 0;
    public static double stalCheckDuration = 100;
    public boolean beamChecked = false;

    @Override
    public void update() {
        stallTriggeredThisLoop = false;

        if(intakeState != IntakeState.INTAKE) beamChecked = false;
        switch (intakeState) {
            case OFF:
                blockerState = BlockerState.CLOSE;
                powerArmState = PowerArmState.INTAKE;
                intake.setPower(0);
                conveyorState = ConveyorState.OFF;
                capacState = CapacState.BLEG;
                if(robot.sensors.lightColor == Sensors.LightColor.BLUE) {
                    robot.sensors.setLedColor(Sensors.LightColor.OFF);
                }
                break;
            case OFF_OPEN:
                blockerState = BlockerState.OPEN;
                powerArmState = PowerArmState.INTAKE;
                intake.setPower(0);
                conveyorState = ConveyorState.OFF;
                capacState = CapacState.BLEG;
                break;
            case INTAKE:
                robot.sensors.setLedColor(Sensors.LightColor.RED);
                blockerState = BlockerState.CLOSE;
                capacState = CapacState.BLEG;
                powerArmState = PowerArmState.INTAKE;
                intake.setPower(IntakeConstants.intakePowerIntake);
                if((robot.sensors.isBreakBeamPos3Low() && robot.sensors.getHowLongBeam3() > IntakeConstants.beam3StopDelay)  || beamChecked) {
                    conveyorState = ConveyorState.OFF;
                    beamChecked = true;
                } else {
                    conveyorState = ConveyorState.ON;
                }

                if(robot.sensors.isBreakBeamPos1Low()
                        && (robot.sensors.getHowLongBeam1()) > IntakeConstants.beamAllStopDelay
                        && (robot.sensors.isBreakBeamPos2Low() && robot.sensors.getHowLongBeam2() > IntakeConstants.beamAllStopDelay)){
                    robot.op.gamepad1.rumble(250);
                    robot.sensors.setLedColor(Sensors.LightColor.GREEN);
                    intakeState = IntakeState.OFF;
                }

                break;
            case REVERSE:
                blockerState = BlockerState.CLOSE;
                intake.setPower(-IntakeConstants.reversePower);
                conveyorState = ConveyorState.REVERSE;
                capacState = CapacState.BLEG;
                break;
            case START_TRANSFER:
                blockerState = BlockerState.OPEN;
                powerArmState = PowerArmState.LOW;
                intake.setPower(IntakeConstants.transferPowerIntake);
                conveyorState = ConveyorState.ON;
                sleep(IntakeConstants.capacReleaseTransfer, IntakeState.TRANSFER);
                capacState = CapacState.BLEG;
                robot.sensors.setLedColor(Sensors.LightColor.BLUE);
                break;
            case POWER_FOR_TIME:
                powerArmState = PowerArmState.LOW;
                intake.setPower(power_time);
                conveyorState = ConveyorState.POWER_FOR_TIME;
                sleep(time_power, IntakeState.OFF_OPEN);
                capacState = CapacState.BLEG;
                break;
            case TRANSFER:
                powerArmState = PowerArmState.TRANSFER;
                intake.setPower(IntakeConstants.transferPowerIntake);
                conveyorState = ConveyorState.ON;
                capacState = CapacState.BLEG;
                break;
            case SLEEP:
                Log.w("Debug shoot precise", " " + (System.currentTimeMillis() - startSleep));

                if (System.currentTimeMillis() - startSleep > sleeptime) {
                    intakeState = nextState;
                }
                break;
            case RECYCLE:
                capacState = CapacState.RECYCLE;
                powerArmState = PowerArmState.RECYCLE;
                intake.setPower(IntakeConstants.intakePowerRecycle);
                conveyorState = ConveyorState.ON;
                blockerState = BlockerState.OPEN;
                break;
        }

        switch (blockerState) {
            case OPEN:
                blocker.setPosition(IntakeConstants.blockerOpen);
                break;
            case CLOSE:
                blocker.setPosition(IntakeConstants.blockerClose);
                break;
        }
        switch (powerArmState) {
            case LOW:
                powerArm.getController().pwmEnable();
                powerArm.setPosition(IntakeConstants.powerArmLow);
                break;
            case INTAKE:
                powerArm.getController().pwmEnable();
                powerArm.setPosition(IntakeConstants.powerArmIntake);
                break;
            case BLEG:
                powerArm.getController().pwmDisable();
                break;
            case RECYCLE:
                powerArm.getController().pwmEnable();
                powerArm.setPosition(IntakeConstants.powerArmRecycle);
                break;
            case TRANSFER:
                powerArm.getController().pwmEnable();
                powerArm.setPosition(IntakeConstants.powerArmVeryLow);
                break;
        }
        switch (capacState){
            case BLEG:
                capac.setPosition(IntakeConstants.capacBleg);
                break;
            case RECYCLE:
                capac.setPosition(IntakeConstants.capacRecycle);
                break;
            case WIGGLEWIGGLEWIGGLE:
                //Wiggle Wiggle Wiggle, du-tu-tu du du du, Wiggle Wiggle Wiggle...
                break;
        }
        switch (conveyorState) {
            case OFF:
                conveyor.setPower(0);
                break;
            case ON:
                conveyor.setPower(IntakeConstants.transferPowerTransfer);
                break;
            case POWER_FOR_TIME:
                conveyor.setPower(power_time);
                break;
            case REVERSE:
                conveyor.setPower(-IntakeConstants.reversePower);
                break;
        }

        //useStall este depricated si nu trebuie folosit sub nici un fel!!!
        if (useStall) {
            boolean ballPresent = robot.sensors != null && robot.sensors.isBreakBeamPos1Low();
            boolean intakeStalled = intake.isOverCurrent();

            switch (stallCheck) {
                case IDLE:
                    intakeSensorCounter = 0;
                    if (intakeState == IntakeState.INTAKE && ballPresent) {
                        intakeSensorCounter = 1;
                        stallCheck = StallCheck.DETECTED;
                    }
                    break;
                case DETECTED:
                    if (intakeState != IntakeState.INTAKE) {
                        stallCheck = StallCheck.IDLE;
                        intakeSensorCounter = 0;
                        break;
                    }
                    if (ballPresent) {
                        intakeSensorCounter++;
                    } else {
                        intakeSensorCounter = 0;
                    }

                    if (intakeSensorCounter >= IntakeConstants.intakeSensorThreshold && intakeStalled) {
                        startStallCheckTime = System.currentTimeMillis();
                        stallCheck = StallCheck.CONFIRMING;
                    }
                    break;
                case CONFIRMING:
                    Log.w("IntakeTransfer", "intake sensor counter = " + intakeSensorCounter + " stalled=" + intakeStalled);
                    if (intakeState != IntakeState.INTAKE) {
                        stallCheck = StallCheck.IDLE;
                        break;
                    }

                    if (!ballPresent || !intakeStalled) {
                        stallCheck = StallCheck.DETECTED;
                        break;
                    }

                    if (System.currentTimeMillis() - startStallCheckTime > stalCheckDuration) {
                        Log.w("IntakeTransfer", "Stall detected, stopping intake");

                        robot.op.gamepad1.rumble(150);
                        stallTriggeredThisLoop = true;
                        setIntakeState(IntakeState.OFF);
                        stallCheck = StallCheck.IDLE;
                    }
                    break;

            }
        }

    }

    public void setIntakeState(IntakeState intakeState) {
        this.intakeState = intakeState;
    }

    public void setBlockerState(BlockerState blockerState) {
        this.blockerState = blockerState;
    }

    public void setpowerArmState(PowerArmState state) {
        this.powerArmState = state;
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
