package org.firstinspires.ftc.teamcode.Hardware.Drive.p2p.localizer;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public class Localization {
    GoBildaPinpointDriver pinpoint;
    public static double offsetX = 0;
    public static double offsetY = 0;
    Pose2D currentPose = new Pose2D(DistanceUnit.INCH,0,0, AngleUnit.RADIANS,0);
    Pose2D lastPose = new Pose2D(DistanceUnit.INCH,0,0, AngleUnit.RADIANS,0);

    public static GoBildaPinpointDriver.EncoderDirection strafeDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection forwardDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    double oldTime = 0;

    public Localization(HardwareMap hw,Pose2D startPose) {
        pinpoint = hw.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setOffsets(offsetX, offsetY, DistanceUnit.MM);

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(strafeDirection, forwardDirection);

        pinpoint.resetPosAndIMU();

        if(startPose != null) {
            pinpoint.setPosition(startPose);
            currentPose = startPose;
        }
    }

    public void update() {
        pinpoint.update();


    }

}
