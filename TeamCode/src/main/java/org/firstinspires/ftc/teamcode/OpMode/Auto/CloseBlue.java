package org.firstinspires.ftc.teamcode.OpMode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Util.Globals.Alliance;
import org.firstinspires.ftc.teamcode.Util.Info;

@Autonomous(name = "Auto Blue Close")
public class CloseBlue extends Close{
    @Override
    public void init() {
        Info.alliance = Alliance.BLUE;

        super.init();
    }

}