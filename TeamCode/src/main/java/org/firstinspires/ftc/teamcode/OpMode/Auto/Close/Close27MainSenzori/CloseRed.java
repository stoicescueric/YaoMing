package org.firstinspires.ftc.teamcode.OpMode.Auto.Close.Close27MainSenzori;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Util.Globals.Alliance;
import org.firstinspires.ftc.teamcode.Util.Info;

@Autonomous(name = "Auto Red 27 (main senzori)")
public class CloseRed extends Close27MainSensors {
    @Override
    public void init() {
        Info.alliance = Alliance.RED;

        super.init();
    }

}