package org.firstinspires.ftc.teamcode.OpMode.Auto.Close.Close24;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Util.Globals.Alliance;
import org.firstinspires.ftc.teamcode.Util.Info;

@Autonomous(name = "Auto Blue 24")
public class CloseBlue extends Close24 {
    @Override
    public void init() {
        Info.alliance = Alliance.BLUE;

        super.init();
    }

}