package org.firstinspires.ftc.teamcode.Util;



import org.firstinspires.ftc.teamcode.Util.Globals.Alliance;
import org.firstinspires.ftc.teamcode.Util.Globals.Phase;


public class Info {
    public static Phase phase = Phase.TELEOP;
    public static boolean useBlob = false;
    public static String opModeName = "";
    public static Alliance alliance = Alliance.RED;
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

    public static double lastPoseX = 0.0;
    public static double lastPoseY = 0.0;
    public static double lastPoseHeading = 0.0;
    public static boolean hasLastPose = false;
}
