package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Util.Globals.Alliance;
import org.firstinspires.ftc.teamcode.Util.Info;

@TeleOp(name = "TeleOp Blue")
public class TeleOpBlue extends TeleOP{
    @Override
    public void runOpMode() throws InterruptedException {
        Info.alliance = Alliance.BLUE;
        super.runOpMode();
    }
}
