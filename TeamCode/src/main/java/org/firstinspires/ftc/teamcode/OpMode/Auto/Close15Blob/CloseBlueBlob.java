package org.firstinspires.ftc.teamcode.OpMode.Auto.Close15Blob;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Util.Globals.Alliance;
import org.firstinspires.ftc.teamcode.Util.Info;

@Autonomous(name = "Auto Blue Close Blob")
public class CloseBlueBlob extends CloseBlob {
    @Override
    public void init() {
        Info.alliance = Alliance.BLUE;

        super.init();
    }

}