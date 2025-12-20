package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Util.Globals.Alliance;
import org.firstinspires.ftc.teamcode.Util.Info;

@TeleOp(name = "TeleOp Red")
public class TeleOpRed extends TeleOP{

    @Override
    public void runOpMode() throws InterruptedException {
        TeleOP.flipFieldFrame = true;
        Info.alliance = Alliance.RED;
        super.runOpMode();
    }
}
