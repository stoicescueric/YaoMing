package org.firstinspires.ftc.teamcode.Hardware.Outtake;


import com.acmerobotics.dashboard.config.Config;

@Config
public class OuttakePositions {
    /*----Launcher ----*/
    public static double FAR_ZONE_X_THRESHOLD = 17;

    /*----Velocity ----*/
    public static double errorVelThreeshold = 30;

    public static double farLaunchVelocity = 2100;
    public static double closeLaunchVelocity = 1750;

    public static double defaultVel = 1470;


    /*----Tilt ----*/
    public static double farLaunchTilt = 0.40;
    public static double closeLaunchTilt = 0.32;

    /*--PIDF VELOCITY--*/
    public static double kP = 0.0045;
    public static double kI = 0;
    public static double kD = 0;
        public static double kV = 0.000365;
    public static double kS = 0.08;

    /*--TURRET--*/
    public static double MIN_TURRET_ANGLE = -3.7802;
    public static double MAX_TURRET_ANGLE = 2.2730;
    public static double MIN_TURRET_POSITION = 0;
    public static double MAX_TURRET_POSITION = 1;


}