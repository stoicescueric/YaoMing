package org.firstinspires.ftc.teamcode.OpMode.robotNou;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Util.Wrapper.GamePadController;

@Config
@TeleOp(name = "TransferTest", group = "Test")
public class TransferTest extends LinearOpMode {
    public static double transferSpeed = 1;
    public static double intakeSpeed = 1;
    GamePadController gg;
    DcMotorEx intake,transfer;
    @Override
    public void runOpMode() throws InterruptedException {
        gg = new GamePadController(gamepad1);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        while(opModeIsActive()){
            gg.update();
            if(gg.a()) {
                intake.setPower(intakeSpeed);
                transfer.setPower(transferSpeed);
            }else if(gg.b()) {
                intake.setPower(intakeSpeed);
            }else {
                intake.setPower(0);
                transfer.setPower(0);
            }
        }

    }
}
