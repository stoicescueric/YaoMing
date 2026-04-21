package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Util.Wrapper.GamePadController;


@Config
@TeleOp(name = "MotorToggleABXY", group = "Pedro Pathing")
public class MotorToggleABXY extends LinearOpMode {
    private DcMotor m1, m2, m3, m4;
    private boolean m1On = false;
    private boolean m2On = false;
    private boolean m3On = false;
    private boolean m4On = false;

    public static double motor1Power = 1.0;
    public static double motor2Power = 1.0;
    public static double motor3Power = 1.0;
    public static double motor4Power = 1.0;


    @Override
    public void runOpMode() throws InterruptedException {
        m1 = hardwareMap.get(DcMotor.class, "fl");
        m2 = hardwareMap.get(DcMotor.class, "bl");
        m3 = hardwareMap.get(DcMotor.class, "fr");
        m4 = hardwareMap.get(DcMotor.class, "br");

        setZeroPowerBrake(m1);
        setZeroPowerBrake(m2);
        setZeroPowerBrake(m3);
        setZeroPowerBrake(m4);

        GamePadController gg = new GamePadController(gamepad1);

        telemetry.addLine("MotorToggleABXY ready. Use A/B/X/Y to toggle m1/m2/m3/m4.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            gg.update();

            if (gg.aOnce()) {
                m1On = !m1On;
            }
            if (gg.bOnce()) {
                m2On = !m2On;
            }
            if (gg.xOnce()) {
                m3On = !m3On;
            }
            if (gg.yOnce()) {
                m4On = !m4On;
            }

            m1.setPower(m1On ? motor1Power : 0.0);
            m2.setPower(m2On ? motor2Power : 0.0);
            m3.setPower(m3On ? motor3Power : 0.0);
            m4.setPower(m4On ? motor4Power : 0.0);

            telemetry.addData("m1", m1On ? motor1Power : 0.0);
            telemetry.addData("m2", m2On ? motor2Power : 0.0);
            telemetry.addData("m3", m3On ? motor3Power : 0.0);
            telemetry.addData("m4", m4On ? motor4Power : 0.0);
            telemetry.update();
        }

        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
    }

    private void setZeroPowerBrake(DcMotor m) {
        try {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception ignored) {}
        try {
            m.setDirection(DcMotorSimple.Direction.FORWARD);
        } catch (Exception ignored) {}
    }
}

