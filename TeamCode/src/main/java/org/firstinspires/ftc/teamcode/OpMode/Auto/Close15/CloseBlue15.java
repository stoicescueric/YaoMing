package org.firstinspires.ftc.teamcode.OpMode.Auto.Close15;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Util.Globals.Alliance;
import org.firstinspires.ftc.teamcode.Util.Info;

@Autonomous(name = "Auto Blue Close 15")
public class CloseBlue15 extends Close15{
    @Override
    public void init() {
        Info.alliance = Alliance.BLUE;

        super.init();
    }

}