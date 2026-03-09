package org.firstinspires.ftc.teamcode.Hardware.Intake;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware.Module;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.OuttakePositions;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Turret;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.Sensors;
import org.firstinspires.ftc.teamcode.Util.Caching.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.Util.Caching.CachingServo;
import org.firstinspires.ftc.teamcode.Util.HardwareUtils;
import org.firstinspires.ftc.teamcode.Util.Utils;
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
        ReCycleStart,
        ReCycleMid,
        ReCycleEnd,
        POWER_FOR_TIME,
        OFF_OPEN,

        SLEEP,
        TRANSFER,
        PRE_OFF_OPEN,
        HOLD,
        RECYCLE
    }

    public enum BlockerState {
        CLOSE,
        OPEN,
        BLOCKER_ACTUALLY_OPEN
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
        REVERSE_LITTLE,
        OFF,
        ON,
        POWER_FOR_TIME,
        TRANSFER,
        recycle1,
        recycle2,
        recycle3,
        reverseTransfer,
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
    public  IntakeState previousState = IntakeState.OFF;
    private boolean shooterStartRecycle = false;

    private long blockerOpenTriggeredTime = 0;

    BinaryDeque deq = new BinaryDeque();
    ElapsedTime pre_off_open = null;

    public IntakeTransfer(Robot robot, Sensors sensors) {
        this.robot = robot;
        intake = new CachingDcMotorEx(robot.hw.get(DcMotorEx.class, "intake"), 0);
        conveyor = new CachingDcMotorEx(robot.hw.get(DcMotorEx.class, "transfer"), 0);

        powerArm = new CachingServo(robot.hw.get(Servo.class, "powerArm"));
        blocker = new CachingServo(robot.hw.get(Servo.class, "blocker"));
        capac = new CachingServo(robot.hw.get(Servo.class, "capac"));
        HardwareUtils.unlock(intake);
        HardwareUtils.unlock(conveyor);
        intake.setCurrentAlert(IntakeConstants.intakeAmpsThreshold, CurrentUnit.AMPS);
        intake.setDirection(IntakeConstants.isIntakeReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        conveyor.setDirection(IntakeConstants.isTransferReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

    }

    private boolean two = false;
    private boolean spinUpRecycleWant = false;
    ElapsedTime recycleStartTimer = new ElapsedTime();
    ElapsedTime recycleMidTimer = new ElapsedTime();
    ElapsedTime recycleEndTimer = new ElapsedTime();
    ElapsedTime intakeSeq = new ElapsedTime();
    public double power_time = 0.5;
    public double time_power = 500;
    public double startStallCheckTime = 0;
    public static double stalCheckDuration = 100;
    public boolean beamChecked = false;

    @Override
    public void update() {

        if(intakeState != IntakeState.INTAKE) beamChecked = false;
        switch (intakeState) {
            case OFF:
                blockerState = BlockerState.CLOSE;
                powerArmState = PowerArmState.INTAKE;
                intake.setPower(IntakeConstants.IntakeLittle);
                conveyorState = ConveyorState.OFF;
                if(robot.sensors.lightColor == Sensors.LightColor.BLUE) {
                    robot.sensors.setLedColor(Sensors.LightColor.OFF);
                }
                break;
            case PRE_OFF_OPEN:
                powerArmState = PowerArmState.INTAKE;
                intake.setPower(IntakeConstants.IntakeLittle);
                conveyorState = ConveyorState.REVERSE_LITTLE;
                capacState = CapacState.BLEG;
                if(pre_off_open == null) {
                    pre_off_open = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
                }
                if(pre_off_open.milliseconds() > 125) {
                    intakeState = IntakeState.OFF_OPEN;
                    pre_off_open = null;
                }
                break;
            case OFF_OPEN:
                blockerState = BlockerState.BLOCKER_ACTUALLY_OPEN;
                powerArmState = PowerArmState.INTAKE;
                intake.setPower(0);
                conveyorState = ConveyorState.REVERSE_LITTLE;

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
                    pre_off_open = null;
                    intakeState = IntakeState.PRE_OFF_OPEN;
                }

                break;
            case REVERSE:
                blockerState = BlockerState.CLOSE;
                intake.setPower(-IntakeConstants.reversePower);
                conveyorState = ConveyorState.REVERSE;
                capacState = CapacState.BLEG;
                break;
            case START_TRANSFER:

                intake.setPower(0);
                powerArmState = PowerArmState.LOW;
                conveyorState = ConveyorState.OFF;
                capacState = CapacState.BLEG;

                robot.sensors.setLedColor(Sensors.LightColor.BLUE);
                blockerState = BlockerState.BLOCKER_ACTUALLY_OPEN;
                sleep(IntakeConstants.sleepTransfer,IntakeState.TRANSFER);

                Log.w("START TRANSFER","previous state: " + previousState + " intakeState " + intakeState);

                break;
            case ReCycleStart:
                robot.outtake.turret.turretState = Turret.TurretState.FIXED_ANGLE;
                if(recycleStartTimer == null){
                    recycleStartTimer = new ElapsedTime();
                }
                capacState = CapacState.RECYCLE;
                powerArmState = PowerArmState.INTAKE;
                intake.setPower(IntakeConstants.intakeFirstPhase);
                conveyorState = ConveyorState.recycle1;
                robot.outtake.launcher.autoAimOn(false);
                robot.outtake.launcher.setTargetHood(1);
                robot.outtake.setOuttakeState(Outtake.OuttakeState.OFF);
                robot.outtake.flywheelSpin(OuttakePositions.recycleSpeed);
                if(robot.outtake.launcher.isReady() && recycleStartTimer.milliseconds() > IntakeConstants.timerRecycleFirstPhase){
                    powerArmState = PowerArmState.INTAKE;
                    intakeState = IntakeState.ReCycleMid;
                    recycleStartTimer = null;
                }
                break;
            case ReCycleMid:
                if(recycleMidTimer == null){
                    recycleMidTimer = new ElapsedTime();
                }

                if(recycleMidTimer.milliseconds() > IntakeConstants.timerRecycleOpenBlocker) {
                    blockerState = BlockerState.BLOCKER_ACTUALLY_OPEN;
                    conveyorState = ConveyorState.recycle2;
                    intake.setPower(IntakeConstants.intakeSecondPhase);
                }
                if(recycleMidTimer.milliseconds() > IntakeConstants.timerRecycleOpenBlocker + IntakeConstants.powerArmRecycleUp) {
                    powerArmState = PowerArmState.RECYCLE;
                }
                if(!two &&
                        recycleMidTimer.milliseconds() > IntakeConstants.timerRecycleOpenBlocker + IntakeConstants.timerRecycleOne){
                    conveyorState = ConveyorState.OFF;
                    intakeState = IntakeState.ReCycleEnd;
                }else if(two && recycleMidTimer.milliseconds() > IntakeConstants.timerRecycleOpenBlocker + IntakeConstants.timerRecycleTwo){
                    conveyorState = ConveyorState.OFF;
                    intakeState = IntakeState.ReCycleEnd;
                }
                break;
            case ReCycleEnd:
                if(recycleEndTimer == null){
                    recycleEndTimer = new ElapsedTime();


                }
                if(shooterStartRecycle) {
                    if(spinUpRecycleWant)
                        robot.outtake.setOuttakeState(Outtake.OuttakeState.READY_FLYWHEEL);
                    else
                        robot.outtake.setOuttakeState(Outtake.OuttakeState.IDLE);
                }
                blockerState = BlockerState.CLOSE;
                intake.setPower(IntakeConstants.intakePhase3);
                conveyorState = ConveyorState.reverseTransfer;
                if(recycleEndTimer.milliseconds() > IntakeConstants.timerIntakeEnd){
                    intake.setPower(IntakeConstants.intakePhase3);
                    conveyorState = ConveyorState.recycle3;
                }
                if(recycleEndTimer.milliseconds() > IntakeConstants.timerIntakeEnd + IntakeConstants.timerIntakeEnd2) {
                    if(!shooterStartRecycle) {
                        shooterStartRecycle = true;
                        robot.outtake.launcher.autoAimOn(true);
                        robot.outtake.turret.turretState = Turret.TurretState.TRACKING;
                    }
                    powerArmState = PowerArmState.TRANSFER;
                }
                if(recycleEndTimer.milliseconds() > IntakeConstants.timerIntakeEnd + IntakeConstants.timerIntakeEnd2 + IntakeConstants.doneTransfer) {
                    intakeState = IntakeState.OFF;
                }

                break;
            case POWER_FOR_TIME:
                powerArmState = PowerArmState.LOW;
                intake.setPower(power_time);
                conveyorState = ConveyorState.POWER_FOR_TIME;
                sleep(time_power, IntakeState.OFF_OPEN);
                capacState = CapacState.BLEG;
                break;
            case TRANSFER:
                conveyor.setPower(Utils.minMaxClip(IntakeConstants.transferPowerTransfer * (12 / robot.sensors.getVoltage()),-1,1));
                intake.setPower(Utils.minMaxClip(IntakeConstants.transferPowerIntake * (12 / robot.sensors.getVoltage()),-1,1));
                powerArmState = PowerArmState.TRANSFER;
                conveyorState = ConveyorState.TRANSFER;
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
                if (blockerOpenTriggeredTime == 0) {
                    blockerOpenTriggeredTime = System.currentTimeMillis();
                }
                if (System.currentTimeMillis() - blockerOpenTriggeredTime >= OuttakePositions.blockerOpenDelayMs) {
                    blockerState = BlockerState.BLOCKER_ACTUALLY_OPEN;
                }
                break;
            case BLOCKER_ACTUALLY_OPEN:
                blocker.setPosition(IntakeConstants.blockerOpen);
                break;
            case CLOSE:
                blockerOpenTriggeredTime = 0;
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
            case REVERSE_LITTLE:
                conveyor.setPower(IntakeConstants.ConveyerLittle);
                break;
            case OFF:
                conveyor.setPower(0);
                break;
            case recycle1:
                conveyor.setPower(IntakeConstants.transferFirstPhase);
                break;
            case recycle2:
                conveyor.setPower(IntakeConstants.transferSecondPhase);
                break;
            case recycle3:
                conveyor.setPower(IntakeConstants.conveyerPhase3);
                break;
            case ON:
                conveyor.setPower(IntakeConstants.onPowerConveyer);
                break;
            case POWER_FOR_TIME:
                conveyor.setPower(power_time);
                break;
            case TRANSFER:
                conveyor.setPower(Utils.minMaxClip(IntakeConstants.transferPowerTransfer * (12 / robot.sensors.getVoltage()),-1,1));
                break;
            case reverseTransfer:
                conveyor.setPower(IntakeConstants.reverseConPhase3);
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
        previousState = intakeState;


    }

    public void startRecycle(boolean two) {
        this.two = two;
        recycleStartTimer = null;
        recycleMidTimer = null;
        recycleEndTimer = null;
        intakeState = IntakeState.ReCycleStart;
        spinUpRecycleWant = false;
        shooterStartRecycle = false;
    }

    public void spinUpRecycle() {
        spinUpRecycleWant = true;
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

    public boolean isRecycle() {
        return (intakeState == IntakeState.ReCycleStart || intakeState == IntakeState.ReCycleMid || intakeState == IntakeState.ReCycleEnd);
    }
    private void sleep(double time, IntakeState nextState) {
        startSleep = System.currentTimeMillis();
        intakeState = IntakeState.SLEEP;
        sleeptime = time;
        this.nextState = nextState;
    }
}
