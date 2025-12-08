package org.firstinspires.ftc.teamcode.Util;



import org.firstinspires.ftc.teamcode.Util.Globals.Alliance;
import org.firstinspires.ftc.teamcode.Util.Globals.Phase;


public class Info {
    public static Phase phase = Phase.TELEOP;
    public static String opModeName = "";
    public Alliance alliance = Alliance.RED;
    public Info() {

    }
    public void setAlliance(Alliance seted) {
        alliance =  seted;
    }
    public Alliance getAllicane() {
        return alliance;
    }
    public void toggleAliance() {
        if(alliance == Alliance.RED) {
            alliance = Alliance.BLUE;
        }else {
            alliance = Alliance.RED;
        }
    }

    public static int cycleCounts = 0;
    public static boolean hasSample = false;
}
