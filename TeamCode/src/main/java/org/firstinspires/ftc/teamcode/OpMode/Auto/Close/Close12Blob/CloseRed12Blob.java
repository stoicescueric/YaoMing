package org.firstinspires.ftc.teamcode.OpMode.Auto.Close.Close12Blob;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Util.Globals.Alliance;
import org.firstinspires.ftc.teamcode.Util.Info;

@Autonomous(name = "Auto Red 12")
public class CloseRed12Blob extends Close12Blob {
    @Override
    public void init() {
        Info.alliance = Alliance.RED;

        super.init();
    }

}
