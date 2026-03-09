package org.firstinspires.ftc.teamcode.OpMode.Auto.Close.Close18Playoff;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Util.Globals.Alliance;
import org.firstinspires.ftc.teamcode.Util.Info;

@Autonomous(name = "Auto Blue 18 Playoff")
public class CloseBlue extends Close18 {
    @Override
    public void init() {
        Info.alliance = Alliance.BLUE;

        super.init();
    }

}