package org.firstinspires.ftc.teamcode.Hardware.Outtake;


import com.acmerobotics.dashboard.config.Config;

@Config
public class OuttakePositions {
    /*----Velocity ----*/
    public static double errorVelThreeshold = 5;

    public static double farLaunchVelocity = 2000;
    public static double closeLaunchVelocity = 1750;


    /*----Tilt ----*/
    public static double farLaunchTilt = 0.40;
    public static double closeLaunchTilt = 0.32;

    /*--PIDF VELOCITY--*/
    public static double kP = 0.00278;
    public static double kI = 0;
    public static double kD = 0;
    public static double kV = 0.00044;
    public static double kS = 0;
}