package org.firstinspires.ftc.teamcode.OpMode.robotNou;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;

import org.firstinspires.ftc.teamcode.Util.Wrapper.GamePadController;

@Config
@TeleOp(name = "Climb Test", group = "robotNou")
public class ClimbTest extends LinearOpMode {
    CRServoImplEx left1,right,right2,left;
    public static double climbPower = 1;
    public static boolean isLeftReversed = false;
    public static boolean isLeft2Reversed = false;
    public static double holdPower = 0.5;
    GamePadController gg;
    public static boolean isRightReversed = false;
    public static boolean isRight2Reversed = false;
    @Override
    public void runOpMode() throws InterruptedException {
        gg = new GamePadController(gamepad1);

      left = hardwareMap.get(CRServoImplEx.class,"climbLeft2");
      right2 = hardwareMap.get(CRServoImplEx.class,"climbRight2");
        left1 = hardwareMap.get(CRServoImplEx.class,"climbLeft");
        right = hardwareMap.get(CRServoImplEx.class,"climbRight");


        waitForStart();
        while (opModeIsActive()){
            gg.update();
            if(gamepad1.a){
                left1.setPower((isLeft2Reversed?-1:1)*climbPower);
                right2.setPower((isRight2Reversed?-1:1)*climbPower);
                left.setPower((isLeftReversed?-1:1)*climbPower);
                right.setPower((isRightReversed?-1:1)*climbPower);
            }
            else if(gamepad1.b){
                left.setPower((isLeftReversed?-1:1)*holdPower);
                right2.setPower((isRight2Reversed?-1:1)*holdPower);
                left1.setPower((isLeft2Reversed?-1:1)*holdPower);
                right.setPower((isRightReversed?-1:1)*holdPower);
            }
            else{
                left.setPower(0);
                right2.setPower(0);
                left1.setPower(0);
                right.setPower(0);
            }
        }
    }
}
