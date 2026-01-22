package org.firstinspires.ftc.teamcode.OpMode.robotNou;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Util.Wrapper.GamePadController;

import java.util.List;

@Config
@TeleOp(name = "Suge o dondera")
public abstract class ParkTest extends LinearOpMode {

    DcMotorEx leftFront,rightFront,leftBack,rightBack;
    DcMotorEx intake;
    GamePadController gg;


    Servo geckoCapac;
    public static double capacPosition = 0.5;
    enum IntakeState {
        ON,
        OFF
    }

    IntakeState intakeState = IntakeState.OFF;
    public static boolean isLeftFrontReversed = true;
    public static boolean isRightFrontReversed = false;
    public static boolean isLeftBackReversed = true;
    public static boolean isRightBackReversed = false;
    List<DcMotorEx> motors;
    public static double intakePower = 1;



    @Override
    public void runOpMode() throws InterruptedException {
        gg = new GamePadController(gamepad1);
        waitForStart();

        setUpMotor();
        while (opModeIsActive()){
            gg.update();
            geckoCapac.setPosition(capacPosition);
            if(gg.xOnce()) {
                if(intakeState == IntakeState.OFF) {
                    intakeState = IntakeState.ON;
                } else {
                    intakeState = IntakeState.OFF;
                }
            }
            switch (intakeState){
                case OFF:
                    intake.setPower(0);
                    break;
                case ON:
                    intake.setPower(intakePower);
                    break;
            }

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            leftFront.setPower(((y + x + rx) / denominator) * (isLeftFrontReversed ? -1 : 1));
            leftBack.setPower(((y - x + rx) / denominator) * (isLeftBackReversed ? -1 : 1));
            rightFront.setPower(((y - x - rx) / denominator) * (isRightFrontReversed ? -1 : 1));
            rightBack.setPower(((y + x - rx) / denominator) * (isRightBackReversed ? -1 : 1));

        }
    }

    void setUpMotor(){
        geckoCapac = hardwareMap.get(Servo.class,"geckoCapac");
        geckoCapac.setPosition(capacPosition);


        leftFront = hardwareMap.get(DcMotorEx.class,"leftFront");
        motors.add(leftFront);
        rightFront = hardwareMap.get(DcMotorEx.class,"rightFront");
        motors.add(rightFront);
        leftBack = hardwareMap.get(DcMotorEx.class,"leftBack");
        motors.add(leftBack);
        rightBack = hardwareMap.get(DcMotorEx.class,"rightBack");
        motors.add(rightBack);


        intake = hardwareMap.get(DcMotorEx.class,"intake");
        motors.add(intake);

        for(DcMotorEx motor : motors) {
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        }

    }
}