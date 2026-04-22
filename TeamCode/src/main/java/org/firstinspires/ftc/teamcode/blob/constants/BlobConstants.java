package org.firstinspires.ftc.teamcode.blob.constants;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.blob.localization.GoBildaPinpointDriver;


@Config
public abstract class BlobConstants {

    //HardwareMap
    public static String leftFrontName = "fl";
    public static String leftBackName = "bl";
    public static String rightFrontName = "fr";
    public static String rightBackName = "br";
    public static String pinpointName = "pinpoint";

    public static double hDefTresh = Math.toRadians(3);
    public static double xDefTresh = 1;
    public static double yDefTresh = 1;

    public static double defaultVelocityThresh = 4;
    public static double defaultTransThresh = 2;
    //PID
    public static double kP = 0.055, kI = 0, kD = 0; //drive pid
    public static double hP = 1.2, hI = 0, hD = 0.11;     //heading pid

    //Deceleration
    public static double xDeceleration = 80, yDeceleration = 90;
    public static double zpam = 2;

    //Constants
    public static double lateralMultiplier = 1.4;
    public static double voltageConstant = 12;

    //Localization
    public static double xOffset = -3.4842519685;
    public static double yOffset = 7.3971;
    public static GoBildaPinpointDriver.EncoderDirection xPodDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection yPodDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.GoBildaOdometryPods podType = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

}