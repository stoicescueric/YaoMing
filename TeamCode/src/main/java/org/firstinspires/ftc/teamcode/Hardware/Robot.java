package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware.Drive.p2p.MecanumDrive;
import org.firstinspires.ftc.teamcode.Hardware.Intake.IntakeTransfer;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Outtake;

public class Robot {
    public HardwareMap hw;
    public IntakeTransfer intakeTransfer;
    public Outtake outtake;
    public Sensors sensors;
    public MecanumDrive drive;
    OpMode op;
    Telemetry telemetry;
    public Robot(OpMode op)
    {
        this.op = op;
        this.hw = op.hardwareMap;
        telemetry = new MultipleTelemetry(op.telemetry, FtcDashboard.getInstance().getTelemetry());
        sensors = new Sensors(this);
        intakeTransfer = new IntakeTransfer(this,sensors);
        outtake = new Outtake(this,sensors);
        drive = new MecanumDrive(this.hw);
    }

    public void update() {
        intakeTransfer.update();
        outtake.update();
        updateTelemetry();
    }

    public void updateTelemetry() {
        telemetry.addLine("--- Robot Telemetry ---");
        telemetry.addData("Intake State", intakeTransfer.intakeState);
        telemetry.addData("Outtake State", outtake.outtakeState);
        telemetry.addData("Drive Mode", drive.driveMode);

        telemetry.addData("Launcher Target ", outtake.launcher.target);
        telemetry.addData("Launcher Velocity ", outtake.launcher.currentVel);

        telemetry.addData("Intake amps",intakeTransfer.motor1.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Intake2 amps",intakeTransfer.motor2.getCurrent(CurrentUnit.AMPS));


        telemetry.update();

    }

}
