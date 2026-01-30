package org.firstinspires.ftc.teamcode.OpMode.Auto.Close.Pedro;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Util.Globals.Alliance;
import org.firstinspires.ftc.teamcode.Util.Info;

@Autonomous(name = "Auto Red Close Pedro")
public class CloseRed extends Close{
    @Override
    public void init() {
        Info.alliance = Alliance.RED;

        super.init();
    }

}
