package org.firstinspires.ftc.teamcode.Hardware.Intake;

import com.acmerobotics.dashboard.config.Config;

@Config
public class IntakeConstants {

    /*-----MOTOR POWERS---*/
    public static double intakePowerIntake = 1;
    public static double transferPowerIntake = 1;
    public static double reversePower = 1;
    public static double intakePowerRecycle = 1;

    public static double transferPowerTransfer = 1;



    public static double preciseShotPower = 1;


    public static double intakeAmpsThreshold = 2.7;
    public static boolean isIntakeReversed = true;
    public static boolean isTransferReversed = true;

    /*----- SERVO POSITIONS----*/

    public static double capacBleg = 0.51;
    public static double capacRecycle = 0.68;

    public static double blockerOpen = 0.75;
    public static double blockerClose = 0.41;

    public static double powerArmLow = 0.41;
    public static double powerArmRecycle = 0.68;
    public static double powerArmIntake = 0.41;
    public static double powerArmVeryLow = 0.41;

    public static double preciseShotDelay = 400;
    public static double beam3StopDelay = 250;
    public static double beamAllStopDelay = 150;

    /*----- SENSOR CONSTANTS----*/
    public static double intakeSensorThreshold = 3;
}