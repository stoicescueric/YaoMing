package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Turret;
import org.firstinspires.ftc.teamcode.Util.Wrapper.TelemetryUtil;
@Config
public class Robot {
    public HardwareMap hw;
    public IntakeTransfer intakeTransfer;
    public Outtake outtake;
    public Sensors sensors;
    public Follower drive;
    public OpMode op;
    public static boolean showTelemetry = false;
    Telemetry telemetry;
    private double loopTime = 0;
    public Robot(OpMode op)
    {
        this.op = op;
        this.hw = op.hardwareMap;
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
        sensors.updateTargetForZone();
        drive.update();
        if(showTelemetry) updateTelemetry();
        double loop = System.nanoTime();
        TelemetryUtil.packet.put("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        TelemetryUtil.sendTelemetry();
    }

    public long getLoopTimeNs() {
        return (long) loopTime;
    }

    public void updateTelemetry() {
        TelemetryUtil.packet.addLine("--- Robot Telemetry ---");
        TelemetryUtil.packet.put("Intake State", intakeTransfer.intakeState);
        TelemetryUtil.packet.put("Outtake State", outtake.outtakeState);
        TelemetryUtil.packet.put("Launcher State", outtake.launcher.launcherState);

        TelemetryUtil.packet.put("Launcher Target ", outtake.launcher.target);
        TelemetryUtil.packet.put("Launcher Velocity ", outtake.launcher.currentVel);

        TelemetryUtil.packet.put("Intake amps",intakeTransfer.motor1.getCurrent(CurrentUnit.AMPS));
        TelemetryUtil.packet.put("Intake2 amps",intakeTransfer.motor2.getCurrent(CurrentUnit.AMPS));

        TelemetryUtil.packet.put("Current X", sensors.getX());
        TelemetryUtil.packet.put("Current Y", sensors.getY());
        TelemetryUtil.packet.put("Current Heading", sensors.getHeading());
        TelemetryUtil.packet.put("voltage",sensors.getVoltage());

        Canvas field = TelemetryUtil.getPacket().fieldOverlay();
        double rxIn = sensors.getX();
        double ryIn = sensors.getY();
        double rh = sensors.getHeading(); // radians
        double headingVecLenIn = 7; // inch
        field.setStroke("#00ccff");
        field.strokeCircle(rxIn, ryIn, 7);
        field.strokeLine(rxIn, ryIn, rxIn + Math.cos(rh) * headingVecLenIn, ryIn + Math.sin(rh) * headingVecLenIn);

        double txIn = sensors.getTargetX();
        double tyIn = sensors.getTargetY();
        double dx = txIn - rxIn;
        double dy = tyIn - ryIn;
        double distIn = Math.hypot(dx, dy);

        field.setStroke("#ff3366");
        field.strokeCircle(txIn, tyIn, 2);
        field.setStroke("#aa33ff");
        field.strokeLine(rxIn, ryIn, txIn, tyIn);

        double normalLen = 10.0;
        field.setStroke("#00ff00");
        double b1mag = Math.hypot(Turret.BOARD1_NXrl, Turret.BOARD1_NYrl);
        double n1x = b1mag > 1e-6 ? Turret.BOARD1_NXrl / b1mag : 0.0;
        double n1y = b1mag > 1e-6 ? Turret.BOARD1_NYrl / b1mag : 0.0;
        field.strokeLine(txIn, tyIn, txIn + n1x * normalLen, tyIn + n1y * normalLen);

        field.setStroke("#ffcc00");
        double b2mag = Math.hypot(Turret.BOARD2_NXrl, Turret.BOARD2_NYrl);
        double n2x = b2mag > 1e-6 ? Turret.BOARD2_NXrl / b2mag : 0.0;
        double n2y = b2mag > 1e-6 ? Turret.BOARD2_NYrl / b2mag : 0.0;
        field.strokeLine(txIn, tyIn, txIn + n2x * normalLen, tyIn + n2y * normalLen);

        field.setStroke("#ffffff");
        double aimLen = 40.0;
        double aimAngle = Turret.lastAdjustedGlobalAngle;
        field.strokeLine(rxIn, ryIn,
                rxIn + Math.cos(aimAngle) * aimLen,
                ryIn + Math.sin(aimAngle) * aimLen);

        TelemetryUtil.packet.put("Distance to Target (in)", distIn);
    }
}
