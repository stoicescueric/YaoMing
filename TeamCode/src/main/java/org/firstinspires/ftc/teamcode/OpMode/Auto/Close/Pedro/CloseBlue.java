package org.firstinspires.ftc.teamcode.OpMode.Auto.Close.Pedro;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Util.Globals.Alliance;
import org.firstinspires.ftc.teamcode.Util.Info;

@Disabled
@Autonomous(name = "Auto Blue Close Pedro")
public class CloseBlue extends Close{
    @Override
    public void init() {
        Info.alliance = Alliance.BLUE;

        super.init();
    }

}