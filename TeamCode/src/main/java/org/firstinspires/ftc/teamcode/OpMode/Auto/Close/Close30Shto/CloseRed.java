package org.firstinspires.ftc.teamcode.OpMode.Auto.Close.Close30Shto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Util.Globals.Alliance;
import org.firstinspires.ftc.teamcode.Util.Info;

@Autonomous(name = "Auto Red 30 la shto")
public class CloseRed extends Close30 {
    @Override
    public void init() {
        Info.alliance = Alliance.BLUE;

        super.init();
    }

}