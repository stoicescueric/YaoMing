package org.firstinspires.ftc.teamcode.OpMode.robotNou;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp(name = "Shooter testNou")
public class TestShooter extends LinearOpMode {

    public static double shooterPower1 = 1;
    public static double shooterPower2 = 1;
    public static boolean isSecondReversed = false;
    DcMotorEx shooter1,shooter2;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        shooter1 = hardwareMap.get(DcMotorEx.class,"shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class,"shooter2");

        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(isSecondReversed) {
            shooter2.setDirection(DcMotorEx.Direction.REVERSE);
        }


        waitForStart();
        while(opModeIsActive()){
            shooter1.setPower(shooterPower1);
            shooter2.setPower(shooterPower2);

            telemetry.addData("shooter1 amps",shooter1.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("shooter2 amps",shooter2.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }

    }
}
