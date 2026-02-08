package org.firstinspires.ftc.teamcode.Hardware;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.Hardware.Intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Launcher;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.TeleOP;
import org.firstinspires.ftc.teamcode.Util.Globals.Phase;
import org.firstinspires.ftc.teamcode.Util.Info;
import org.firstinspires.ftc.teamcode.blob.driveTrain.Blob;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Intake.IntakeTransfer;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Turret;
import org.firstinspires.ftc.teamcode.Util.Wrapper.TelemetryUtil;

import java.util.List;

@Config
public class Robot {
    public HardwareMap hw;
    public IntakeTransfer intakeTransfer;
    public Outtake outtake;
    public Sensors sensors;
    public Follower drive = null;
    public Blob blob = null;
    public OpMode op;
    public static boolean showTelemetry = false;
    Telemetry telemetry;
    private double loopTime = 0;
    private DcMotorEx flDriveMotor = null;
    private DcMotorEx blDriveMotor = null;
    private DcMotorEx frDriveMotor = null;
    private DcMotorEx brriveMotor = null;

    LynxModule cHub;

    public Robot(OpMode op)
    {
        cHub = op.hardwareMap.get(LynxModule.class, "Control Hub");

        cHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        this.op = op;
        this.hw = op.hardwareMap;
        if(Info.phase == Phase.AUTONOMOUS && Info.useBlob) {
            blob = new Blob(op.hardwareMap);


        }
        else {
            drive = Constants.createFollower(op.hardwareMap);
        }
        sensors = new Sensors(this);
        intakeTransfer = new IntakeTransfer(this,sensors);
        outtake = new Outtake(this,sensors);
        TelemetryUtil.setup();
    }

    private void ensureFlMotor() {
        if (flDriveMotor == null && hw != null) {
            try {
                flDriveMotor = hw.get(DcMotorEx.class, "leftFront");
                blDriveMotor = hw.get(DcMotorEx.class, "leftBack");
                frDriveMotor = hw.get(DcMotorEx.class, "rightFront");
                brriveMotor = hw.get(DcMotorEx.class, "rightBack");
            } catch (Exception ignored) {
            }
        }
    }

    public void update() {
        cHub.clearBulkCache();
        if(Info.phase == Phase.TELEOP || !Info.useBlob) drive.update();
        else {
            drive = null;
            blob.update();
        }
        sensors.update();
        sensors.updateTargetForZone();
        intakeTransfer.update();
        outtake.update();

        Info.lastPoseX = sensors.getX();
        Info.lastPoseY = sensors.getY();
        Info.lastPoseHeading = sensors.getHeading();
        if (!Info.hasLastPose) Info.hasLastPose = true;

        if(showTelemetry) updateTelemetry();
        double loop = System.nanoTime();
        TelemetryUtil.packet.put("hz ", 1000000000 / (loop - loopTime));
//        TelemetryUtil.packet.put("pinpoint heading", Math.toDegrees(sensors.getHeading()));
//        TelemetryUtil.packet.put("navx heading", sensors.getNavxHeading());

//        double distShootIn = sensors.getDistanceToTarget(sensors.getTargetX(), sensors.getTargetY());
//        TelemetryUtil.packet.put("Distance to Shooting Target (in)", distShootIn);


        loopTime = loop;
        TelemetryUtil.sendTelemetry();

//        if (op != null && op.telemetry != null) {
//            op.telemetry.addData("Distance to Shooting Target (in)", distShootIn);
//            op.telemetry.addData("Shooting While Moving", org.firstinspires.ftc.teamcode.OpMode.TeleOp.TeleOP.shootWhileMoving);
//            op.telemetry.update();
//        }
    }

    public long getLoopTimeNs() {
        return (long) loopTime;
    }

    public void updateTelemetry() {
        // Make sure FL motor reference is resolved if possible
        ensureFlMotor();

        TelemetryUtil.packet.addLine("--- Robot Telemetry ---");
        TelemetryUtil.packet.put("Intake State", intakeTransfer.intakeState);
        TelemetryUtil.packet.put("Outtake State", outtake.outtakeState);
        TelemetryUtil.packet.put("Launcher State", outtake.launcher.launcherState);
        TelemetryUtil.packet.put("Turret State", outtake.turret.turretState);

        TelemetryUtil.packet.put("Launcher Target ", outtake.launcher.target);
        TelemetryUtil.packet.put("Launcher Velocity ", outtake.launcher.currentVel);

        TelemetryUtil.packet.put("Intake amps",intakeTransfer.intake.getCurrent(CurrentUnit.AMPS));
        TelemetryUtil.packet.put("Intake2 amps",intakeTransfer.intake.getCurrent(CurrentUnit.AMPS));
        if (flDriveMotor != null) {
            TelemetryUtil.packet.put("FL Motor amps", flDriveMotor.getCurrent(CurrentUnit.AMPS));
            TelemetryUtil.packet.put("BL Motor amps", blDriveMotor.getCurrent(CurrentUnit.AMPS));
            TelemetryUtil.packet.put("BR Motor amps", brriveMotor.getCurrent(CurrentUnit.AMPS));
            TelemetryUtil.packet.put("FR Motor amps", frDriveMotor.getCurrent(CurrentUnit.AMPS));
        }

        for (DcMotorEx m : hw.getAll(DcMotorEx.class)){
            TelemetryUtil.packet.put(m.getDeviceName() + " amps", m.getCurrent(CurrentUnit.AMPS));
        }

        TelemetryUtil.packet.put("Current X", sensors.getX());
        TelemetryUtil.packet.put("Current Y", sensors.getY());
        TelemetryUtil.packet.put("voltage",sensors.getVoltage());
        TelemetryUtil.packet.put("beam 1 value",sensors.isBreakBeamPos1Low());
        TelemetryUtil.packet.put("beam 2 value",sensors.isBreakBeamPos2Low());
        TelemetryUtil.packet.put("beam 3 value",sensors.isBreakBeamPos3Low());
        TelemetryUtil.packet.put("beam 3 first update",sensors.firstTrueBeam3);
        TelemetryUtil.packet.put("Sensor light",sensors.lightColor.toString());
        TelemetryUtil.packet.put("Hood Target", outtake.launcher.getHoodPosition());
        Canvas field = TelemetryUtil.getPacket().fieldOverlay();
        double rxIn = sensors.getX();
        double ryIn = sensors.getY();
        double rh = sensors.getHeading(); // radians
        double headingVecLenIn = 7; // inch
        field.setStroke("#00ccff");
        field.strokeCircle(rxIn, ryIn, 7);
        field.strokeLine(rxIn, ryIn, rxIn + Math.cos(rh) * headingVecLenIn, ryIn + Math.sin(rh) * headingVecLenIn);

        double shootTx = sensors.getTargetX();
        double shootTy = sensors.getTargetY();
        double backTx = sensors.getBackboardX();
        double backTy = sensors.getBackboardY();

        field.setStroke("#ff3366");
        field.strokeCircle(shootTx, shootTy, 2);
        field.setStroke("#ffa500");
        field.strokeCircle(backTx, backTy, 3);
        field.setStroke("#aa33ff");
        field.strokeLine(rxIn, ryIn, backTx, backTy);

        double distBackIn = sensors.getDistanceToBackboard();
        field.setStroke("#00ff00");

        double normalLen = 10.0;
        double b1mag = Math.hypot(outtake.turret.BOARD1_NXrl, outtake.turret.BOARD1_NYrl);
        double n1x = b1mag > 1e-6 ? outtake.turret.BOARD1_NXrl / b1mag : 0.0;
        double n1y = b1mag > 1e-6 ? outtake.turret.BOARD1_NYrl / b1mag : 0.0;
        field.strokeLine(backTx, backTy, backTx + n1x * normalLen, backTy + n1y * normalLen);

        field.setStroke("#ffcc00");
        double b2mag = Math.hypot(outtake.turret.BOARD2_NXrl, outtake.turret.BOARD2_NYrl);
        double n2x = b2mag > 1e-6 ? outtake.turret.BOARD2_NXrl / b2mag : 0.0;
        double n2y = b2mag > 1e-6 ? outtake.turret.BOARD2_NYrl / b2mag : 0.0;
        field.strokeLine(backTx, backTy, backTx + n2x * normalLen, backTy + n2y * normalLen);

        double aimLen = 40.0;
        double nominalAngle = Turret.lastAdjustedGlobalAngle;
        field.setStroke("#ffffff");
        field.strokeLine(rxIn, ryIn,
                rxIn + Math.cos(nominalAngle) * aimLen,
                ryIn + Math.sin(nominalAngle) * aimLen);

        double compAngle = Turret.lastMotionCompensatedAngle;
        if (Math.abs(compAngle - nominalAngle) > 1e-3) {
            field.setStroke("#00ffff");
            field.strokeLine(rxIn, ryIn,
                    rxIn + Math.cos(compAngle) * aimLen,
                    ryIn + Math.sin(compAngle) * aimLen);
        }

        TelemetryUtil.packet.put("Distance to Backboard (in)", distBackIn);
        double distShootIn2 = sensors.getDistanceToTarget(shootTx, shootTy);
        TelemetryUtil.packet.put("Distance to Shooting Target (in)", distShootIn2);

        TelemetryUtil.packet.put("Robot velX", sensors.getVelX());
        TelemetryUtil.packet.put("Robot velY", sensors.getVelY());

    }
}
