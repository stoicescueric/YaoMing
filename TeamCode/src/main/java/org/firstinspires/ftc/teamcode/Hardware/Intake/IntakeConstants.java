package org.firstinspires.ftc.teamcode.Hardware.Intake;

import com.acmerobotics.dashboard.config.Config;

@Config
public class IntakeConstants {

    /*-----MOTOR POWERS---*/
    public static double intakePowerConveyer = 1;
    public static double intakeShushi = 0.2;
    public static double reversePower = 1;
    public static double transferPowerConveyer = 1;
    public static double transferPowerSushi = 1;

    public static double preciseShotPower = 1;

    public static double leftTransfer = 0.5;
    public static double rightTransfer = 0.5;


    public static double intakeAmpsThreshold = 2.7;
    public static boolean isConveyer1Reversed = true;
    public static boolean isConveyer2Reversed = false;

    /*----- SERVO POSITIONS----*/

    public static double rampOpen = 0.05;
    public static double rampClose = 0.95;

    public static double intakeServoLow = 0.78;
    public static double intakeServoHigh = 0.85;
    public static double intakeServoIntake = 0.75;

    public static double preciseShotDelay = 400;

    /*----- SENSOR CONSTANTS----*/
    public static double intakeSensorThreshold = 3;
}