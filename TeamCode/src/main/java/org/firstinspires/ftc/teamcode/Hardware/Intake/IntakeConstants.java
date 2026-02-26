package org.firstinspires.ftc.teamcode.Hardware.Intake;

import com.acmerobotics.dashboard.config.Config;

@Config
public class IntakeConstants {

    /*-----MOTOR POWERS---*/
    public static double intakePowerIntake = 1;
    public static double transferPowerIntake = 0.5;
    public static double reversePower = 1;
    public static double intakePowerRecycle = 1;

    public static double transferPowerTransfer = 0.85;
    public static double intakeFirstPhase = 0.7;
    public static double intakeSecondPhase = 0.5;
    public static double transferFirstPhase = 0.3;
    public static double transferSecondPhase = 0.5;
    public static double onPowerConveyer = 1;

    public static double timerRecycleOne = 230;
    public static double timerRecycleTwo = 460;
    public static double powerArmRecycleUp = 100;



    public static double preciseShotPower = 1;


    public static double intakeAmpsThreshold = 2.7;
    public static boolean isIntakeReversed = true;
    public static boolean isTransferReversed = true;

    /*----- SERVO POSITIONS----*/

    public static double capacBleg = 0.55;
    public static double capacRecycle = 0.66;
    public static double capacReleaseTransfer = 350;

    public static double blockerOpen = 0.42;
    public static double blockerClose = 0.83;

    public static double powerArmLow = 0.83;
    public static double powerArmRecycle = 0.83;
    public static double powerArmIntake = 0.83;
    public static double powerArmVeryLow = 0.83;

    public static double preciseShotDelay = 400;
    public static double beam3StopDelay = 300;
    public static double beamAllStopDelay = 125;

    public static double timerRecycleFirstPhase = 150;
    public static double timerRecycleOpenBlocker = 200;
    public static double doneTransfer = 150;
    public static double timerIntakeEnd= 250;
    public static double intakeStartTransfer = 100;
    public static double timerIntakeEnd2= 400;
    public static double intakePhase3 = 1;
    public static double conveyerPhase3 = 1;
    public static double reverseConPhase3 = 0.5;

    /*----- SENSOR CONSTANTS----*/
    public static double intakeSensorThreshold = 3;
}