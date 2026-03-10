package org.firstinspires.ftc.teamcode.OpMode.Auto.Close.Close18Playoff;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Util.Globals.Alliance;
import org.firstinspires.ftc.teamcode.Util.Info;

@Config
public class CloseConstants18Playoff {

    //DEFAULT VALUES FOR RED
    public static double turretTargetPos = 0.325;
    public static double shootingTime = 625;
    public static double turretPositionRed = 0.598;
    public static double turretPositionBlue = 0.39;
    public static double failSafeDtTime = 2500;
    public static double hoodPosition = 0.615;
    public static double preloadSotmPerc = 0.35;
    public static double launcherVelocity = 1000;
    public static double startX = -60, startY = 35.7, headingStartRed = Math.toRadians(180); //TODO
    public Pose startPose;
    public static double shootingX = -1, shootingY = 14.5, shootingHeading = Math.toRadians(270);
    public static double shootPreloadX = -14.5,shootPreloadY = 19,shootPreloadH = Math.toRadians(220);
    public static double shootingInterX = -9, shootingInterY = 15.28, shootingUnterHeading = Math.toRadians(270);
    public Pose scorePose;
    public Pose scorePoseGateInter;

    public static double pickUp1X = -13, pickUp1Y = 52, pickUp1Heading = Math.toRadians(270);
    public static double InterpickUp1X = -13, InterpickUp1Y = 25, InterpickUp1Heading = Math.toRadians(270);
    public Pose pickUpPose;
    public Pose pickUpPose1_Inter;
    public static double max_power_pickUp = 1;
    public static double pickUp2X = 9.5,pickUp2Y = 25,     pickUp2Heading = Math.toRadians(270);
    public static double pickUp2X2 = 9.5,pickUp2Y2 = 53, pickUp2Heading2 = Math.toRadians(270);
    public Pose pickUpPose2;
    public Pose pickUpPose2_2;

    public static double pickUp3X = 36,pickUp3Y = 26, pickUp3Heading = Math.toRadians(270);
    public static double pickUp3X2 = 36,pickUp3Y2 = 57, pickUp3Heading2 = Math.toRadians(270);
    public Pose pickUpPose3;
    public Pose pickUpPose3_2;
    public static double parkThreeshold = 29000;

    //gate
    public static long failSafePickupTime = 1900;
    public static int gateCycleCount = 4;
    public static double gatePickupX = 13.5, gatePickupY = 58.6, gatePickupHeading = 4.310019301402063;
    public Pose gatePickupPose;
    public static double percentage = 0.75;

    //clear gate
    public static double clearX = -1, clearY = 56.2, clearHeading = Math.toRadians(90)-Math.PI;
    public static double clearXInter = 2, clearYInter = 46, clearHeadingInter = Math.toRadians(270);
    public Pose clearInter;



    public static double parkX = -3.5, parkY = 24.7, parkHeading = Math.toRadians(90)-Math.PI;

    public Pose parkPose;
    public Pose clear;

    public Pose preload;

    public void buildPaths() {

        parkPose = new Pose(parkX, parkY * (Info.alliance == Alliance.RED ? 1 : -1), parkHeading * (Info.alliance == Alliance.RED ? 1 : -1));
        startPose = new Pose(startX, startY * (Info.alliance == Alliance.RED ? 1 : -1),headingStartRed * (Info.alliance == Alliance.RED ? 1 : -1) );
        scorePose = new Pose(shootingX, shootingY * (Info.alliance == Alliance.RED ? 1 : -1), shootingHeading * (Info.alliance == Alliance.RED ? 1 : -1));
        scorePoseGateInter = new Pose(shootingInterX, shootingInterY * (Info.alliance == Alliance.RED ? 1 : -1), shootingUnterHeading * (Info.alliance == Alliance.RED ? 1 : -1));
        clearInter = new Pose(clearXInter, clearYInter * (Info.alliance == Alliance.RED ? 1 : -1), clearHeadingInter * (Info.alliance == Alliance.RED ? 1 : -1));
        preload = new Pose(shootPreloadX, shootPreloadY * (Info.alliance == Alliance.RED ? 1 : -1), shootPreloadH * (Info.alliance == Alliance.RED ? 1 : -1));

        pickUpPose = new Pose(pickUp1X, pickUp1Y * (Info.alliance == Alliance.RED ? 1 : -1), pickUp1Heading * (Info.alliance == Alliance.RED ? 1 : -1));
        pickUpPose1_Inter = new Pose(InterpickUp1X, InterpickUp1Y * (Info.alliance == Alliance.RED ? 1 : -1), InterpickUp1Heading * (Info.alliance == Alliance.RED ? 1 : -1));
        pickUpPose2 = new Pose(pickUp2X, pickUp2Y * (Info.alliance == Alliance.RED ? 1 : -1), pickUp2Heading * (Info.alliance == Alliance.RED ? 1 : -1));
        pickUpPose2_2 = new Pose(pickUp2X2, pickUp2Y2 * (Info.alliance == Alliance.RED ? 1 : -1), pickUp2Heading2 * (Info.alliance == Alliance.RED ? 1 : -1));
        pickUpPose3 = new Pose(pickUp3X, pickUp3Y * (Info.alliance == Alliance.RED ? 1 : -1), pickUp3Heading * (Info.alliance == Alliance.RED ? 1 : -1));
        pickUpPose3_2 = new Pose(pickUp3X2, pickUp3Y2 * (Info.alliance == Alliance.RED ? 1 : -1), pickUp3Heading2 * (Info.alliance == Alliance.RED ? 1 : -1));
        gatePickupPose = new Pose(gatePickupX, gatePickupY * (Info.alliance == Alliance.RED ? 1 : -1), gatePickupHeading * (Info.alliance == Alliance.RED ? 1 : -1));
        clear = new Pose(clearX, clearY * (Info.alliance == Alliance.RED ? 1 : -1), clearHeading * (Info.alliance == Alliance.RED ? 1 : -1));


    }

    public double getLauncherVelocity() {
        return launcherVelocity;
    }

    public double getTurretPosition() {
        if(Info.alliance == Alliance.RED) {
            return turretPositionRed;
        }else {
            return turretPositionBlue;
        }
    }

    public double getHoodPosition() {
        return hoodPosition;
    }

    public double getShootingTime() {
        return shootingTime;
    }
    public double getMaxPower(){
        return max_power_pickUp;
    }
    public double getFailSafeDtTime() {
        return failSafeDtTime;
    }
    public double getTurretTargetPos() {
        return turretTargetPos;
    }
    public double getFailSafePickupTime(){
        return failSafePickupTime;
    }
    public double getParkThreeshold() {
        return parkThreeshold;
    }
    public double getPreloadProgress() {
        return preloadSotmPerc;
    }


}
