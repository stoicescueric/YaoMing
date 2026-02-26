package org.firstinspires.ftc.teamcode.blob.constants;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.blob.localization.GoBildaPinpointDriver;


@Config
public abstract class BlobConstants {

    //HardwareMap
    public static String leftFrontName = "fr";
    public static String leftBackName = "bl";
    public static String rightFrontName = "fr";
    public static String rightBackName = "br";
    public static String pinpointName = "pinpoint";

    //PID
    public static double kP = 0.08, kI = 0, kD = 0.001; //drive pid
    public static double hP = 0.6, hI = 0, hD = 0.02;     //heading pid

    //Deceleration
    public static double xDeceleration = 30.6366666667, yDeceleration = 57;
    public static double zpam = 3;

    //Constants
    public static double lateralMultiplier = 2.5;
    public static double voltageConstant = 12;

    //Localization
    public static double xOffset = -88.5;
    public static double yOffset = 109.35;
    public static GoBildaPinpointDriver.EncoderDirection xPodDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection yPodDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.GoBildaOdometryPods podType = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

}