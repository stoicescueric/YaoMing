package org.firstinspires.ftc.teamcode.Hardware.Outtake;


import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Hardware.Intake.IntakeTransfer;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.Sensors;

public class Outtake {
    Robot robot;
    public Launcher launcher;
    public Turret turret;
    public enum OuttakeState {
        IDLE,
        SLEEP,
        START_FEEDING,
        LAUNCHING,
        STOP
    }
    public OuttakeState outtakeState = OuttakeState.IDLE;
    public Outtake(Robot robot, Sensors sensors){
        this.robot = robot;
        turret = new Turret(robot.hw,sensors);
        launcher = new Launcher(robot.hw,sensors);

    }
    public void start_feed(double target,double hood) {
        launcher.setTarget(target,hood);
        outtakeState = OuttakeState.START_FEEDING;
    }

    public void update() {
        switch (outtakeState) {
            case IDLE:
                launcher.launcherState = Launcher.LauncherState.OFF;
                break;
            case START_FEEDING:
                if(launcher.isReady()) {
                    robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.START_TRANSFER);
                    outtakeState = OuttakeState.LAUNCHING;
                }
                break;
            case LAUNCHING:
                break;
            case STOP:
                launcher.launcherState = Launcher.LauncherState.OFF;
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF);
                outtakeState = OuttakeState.IDLE;
                break;

        }

        launcher.update();
        turret.update();
    }

    public DcMotorEx getShooterMotor() {
        return launcher.motor1;
    }
}
