package org.firstinspires.ftc.teamcode.blob.constants;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.blob.localization.GoBildaPinpointDriver;


@Config
public abstract class BlobConstants {

    //HardwareMap
    public static String leftFrontName = "m0";
    public static String leftBackName = "m2";
    public static String rightFrontName = "m1";
    public static String rightBackName = "m3";
    public static String pinpointName = "pinpoint";

    //PID
    public static double kP = 0.06, kI = 0, kD = 0; //drive pid
    public static double hP = 1.73, hI = 0, hD = 0.08;     //heading pid

    //Deceleration
    public static double xDeceleration = 70, yDeceleration = 70;
    public static double zpam = 3;

    //Constants
    public static double lateralMultiplier = 2.5;
    public static double voltageConstant = 12;

    //Localization
    public static double xOffset = -82.055;
    public static double yOffset = 103.104;
    public static GoBildaPinpointDriver.EncoderDirection xPodDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    public static GoBildaPinpointDriver.EncoderDirection yPodDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    public static GoBildaPinpointDriver.GoBildaOdometryPods podType = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

}
