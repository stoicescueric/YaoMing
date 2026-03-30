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
    //PID
    public static double kP = 0.07, kI = 0, kD = 0.0079; //drive pid
    public static double hP = 1.9, hI = 0, hD = 0.11;     //heading pid

    //Deceleration
    public static double xDeceleration = 30.6366666667, yDeceleration = 57;
    public static double zpam = 2;

    //Constants
    public static double lateralMultiplier = 1.3;
    public static double voltageConstant = 12;

    //Localization
    public static double xOffset = -3.4842519685;
    public static double yOffset = 4.3031496063;
    public static GoBildaPinpointDriver.EncoderDirection xPodDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection yPodDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.GoBildaOdometryPods podType = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

}