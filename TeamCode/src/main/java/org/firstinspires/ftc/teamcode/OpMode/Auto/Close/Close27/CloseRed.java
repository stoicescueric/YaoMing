package org.firstinspires.ftc.teamcode.OpMode.Auto.Close.Close27;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Util.Globals.Alliance;
import org.firstinspires.ftc.teamcode.Util.Info;

@Autonomous(name = "Auto Red 27")
public class CloseRed extends Close27 {
    @Override
    public void init() {
        Info.alliance = Alliance.RED;

        super.init();
    }

}