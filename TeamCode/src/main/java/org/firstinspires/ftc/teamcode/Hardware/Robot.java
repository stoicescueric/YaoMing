package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Intake.IntakeTransfer;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Util.Wrapper.TelemetryUtil;

public class Robot {
    public HardwareMap hw;
    public IntakeTransfer intakeTransfer;
    public Outtake outtake;
    public Sensors sensors;
    public Follower drive;
    OpMode op;
    Telemetry telemetry;
    public Robot(OpMode op)
    {
        this.op = op;
        this.hw = op.hardwareMap;
        telemetry = new MultipleTelemetry(op.telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = Constants.createFollower(hw);
        sensors = new Sensors(this);
        intakeTransfer = new IntakeTransfer(this,sensors);
        outtake = new Outtake(this,sensors);
        TelemetryUtil.setup();
    }

    public void update() {
        intakeTransfer.update();
        outtake.update();
        sensors.update();
        updateTelemetry();
    }

    public void updateTelemetry() {
        telemetry.addLine("--- Robot Telemetry ---");
        telemetry.addData("Intake State", intakeTransfer.intakeState);
        telemetry.addData("Outtake State", outtake.outtakeState);
        telemetry.addData("Launcher Target ", outtake.launcher.target);
        telemetry.addData("Launcher Velocity ", outtake.launcher.currentVel);
        telemetry.addData("launcher power",outtake.launcher.getPower());

        telemetry.addData("Intake amps",intakeTransfer.motor1.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Intake2 amps",intakeTransfer.motor2.getCurrent(CurrentUnit.AMPS));

        telemetry.addData("Current X", sensors.getX());
        telemetry.addData("Current Y", sensors.getY());
        telemetry.addData("Current Heading", sensors.getHeading());
        telemetry.addData("voltage",sensors.getVoltage());

        Canvas field = TelemetryUtil.getPacket().fieldOverlay();
        double rxIn = sensors.getX();
        double ryIn = sensors.getY();
        double rh = sensors.getHeading(); // radians
        double headingVecLenIn = 7; // inch
        field.setStroke("#00ccff");
        field.strokeCircle(rxIn, ryIn, 7);
        field.strokeLine(rxIn, ryIn, rxIn + Math.cos(rh) * headingVecLenIn, ryIn + Math.sin(rh) * headingVecLenIn);
        field.setStroke("#ff3366");
        double txIn = sensors.getTargetX();
        double tyIn = sensors.getTargetY();
        field.strokeCircle(txIn, tyIn, 2);
        field.setStroke("#aa33ff");
        field.strokeLine(rxIn, ryIn, txIn, tyIn);

        telemetry.addData("Hub Cycle Rate (Hz)", sensors.getCycleRateHz());

        //to delete
        double dx = txIn - rxIn;
        double dy = tyIn - ryIn;
        double distIn = Math.hypot(dx, dy);
        telemetry.addData("Distance to Target (in)", distIn);

        TelemetryUtil.sendTelemetry();
        telemetry.update();
    }
}
