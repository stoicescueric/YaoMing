package org.firstinspires.ftc.teamcode.Hardware.Outtake;


import com.acmerobotics.dashboard.config.Config;

@Config
public class OuttakePositions {
    /*----Launcher ----*/
    public static double FAR_ZONE_X_THRESHOLD = 17;

    /*----Velocity ----*/
    public static double errorVelThreeshold = 15;

    public static double farLaunchVelocity = 2100;
    public static double idleVelocity = 1400;
    public static double idlePower = 0.35;

    public static double defaultVel = 1470;


    /*----Tilt ----*/
    public static double farLaunchTilt = 0.40;

    /*--PIDF VELOCITY--*/
    public static double kP = 0.006;
    public static double kI = 0;
    public static double kD = 0;
    public static double kV = 0.00035;
    public static double kS = 0.06;

    /*--TURRET--*/
    public static double MIN_TURRET_ANGLE = -2.515;
    public static double MAX_TURRET_ANGLE = 2.572;
    public static double MIN_TURRET_POSITION = 0.08;
    public static double MAX_TURRET_POSITION = 0.92;

    /*--BLOCKER--*/
    public static long blockerOpenDelayMs = 160;

}