package org.firstinspires.ftc.teamcode.Hardware.Intake;

import com.acmerobotics.dashboard.config.Config;

@Config
public class IntakeConstants {

    /*-----MOTOR POWERS---*/
    public static double intakePower = 1;
    public static double reversePower = 1;
    public static double transferPower = 1;
    public static double holdPower = 1;
    public static double preciseShotPower = 1;


    public static double intakeAmpsThreshold = 7.5;
    public static boolean isConveyer1Reversed = true;
    public static boolean isConveyer2Reversed = false;

    /*----- SERVO POSITIONS----*/

    public static double rampOpen = 0.05;
    public static double rampClose = 0.95;

    public static double intakeServoLow = 0.64;
    public static double intakeServoHigh = 0.85;

    public static double preciseShotDelay = 400;
}