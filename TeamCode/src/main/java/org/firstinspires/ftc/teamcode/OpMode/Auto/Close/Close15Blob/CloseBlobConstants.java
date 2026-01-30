package org.firstinspires.ftc.teamcode.OpMode.Auto.Close.Close15Blob;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Util.Globals.Alliance;
import org.firstinspires.ftc.teamcode.Util.Info;

@Config
public class CloseBlobConstants {

    //DEFAULT VALUES FOR RED
    public static double shootingTime = 1000;
    public static double turretPositionRed = 0.598;
    public static double turretPositionBlue = 0.39;
    public static double failSafeDtTime = 2500;
    public static double hoodPosition = 0.615;
    public static double launcherVelocity = 1000;
    public static double startX = -62.22, startY = 37.00, headingStartRed =  Math.toRadians(0);
    public Pose startPose;
    public static double shootingX = -4.1, shootingY = 14.5, shootingHeading = Math.toRadians(270);
    public static double shootingInterX = -9, shootingInterY = 15.28, shootingUnterHeading = Math.toRadians(270);
    public Pose scorePose;
    public Pose scorePoseGateInter;

    public static double pickUp1X = -11.3, pickUp1Y = 52, pickUp1Heading = Math.toRadians(270);
    public Pose pickUpPose;
    public static double max_power_pickUp = 1;
    public static double pickUp2X = 13.38,pickUp2Y = 30, pickUp2Heading = Math.toRadians(270);
    public static double pickUp2X2 = 13.6,pickUp2Y2 = 49.4, pickUp2Heading2 = Math.toRadians(270);
    public Pose pickUpPose2;
    public Pose pickUpPose2_2;
    public Pose pickUpPose2Intermediary;
    public static double parkThreeshold = 29000;

    //gate
    public static long failSafePickupTime = 2300;
    public static int gateCycleCount = 3;
    public static double gatePickupX = 12.5, gatePickupY = 59.8, gatePickupHeading = Math.toRadians(240);
    public Pose gatePickupPose;
    public static double percentage = 0.75;

    //clear gate
    public static double clearX = -0.5, clearY = 54, clearHeading = Math.toRadians(90)-Math.PI;


    public static double parkX = -3.5, parkY = 24.7, parkHeading = Math.toRadians(90)-Math.PI;

    public Pose parkPose;
    public Pose clear;



    public void buildPaths(Follower follower) {

        parkPose = new Pose(parkX, parkY * (Info.alliance == Alliance.RED ? 1 : -1), parkHeading * (Info.alliance == Alliance.RED ? 1 : -1));
        startPose = new Pose(startX, startY * (Info.alliance == Alliance.RED ? 1 : -1),headingStartRed * (Info.alliance == Alliance.RED ? 1 : -1) );
        scorePose = new Pose(shootingX, shootingY * (Info.alliance == Alliance.RED ? 1 : -1), shootingHeading * (Info.alliance == Alliance.RED ? 1 : -1));
        scorePoseGateInter = new Pose(shootingInterX, shootingInterY * (Info.alliance == Alliance.RED ? 1 : -1), shootingUnterHeading * (Info.alliance == Alliance.RED ? 1 : -1));

        pickUpPose = new Pose(pickUp1X, pickUp1Y * (Info.alliance == Alliance.RED ? 1 : -1), pickUp1Heading * (Info.alliance == Alliance.RED ? 1 : -1));
        pickUpPose2 = new Pose(pickUp2X, pickUp2Y * (Info.alliance == Alliance.RED ? 1 : -1), pickUp2Heading * (Info.alliance == Alliance.RED ? 1 : -1));
        pickUpPose2_2 = new Pose(pickUp2X2, pickUp2Y2 * (Info.alliance == Alliance.RED ? 1 : -1), pickUp2Heading2 * (Info.alliance == Alliance.RED ? 1 : -1));
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
    public double getFailSafePickupTime(){
        return failSafePickupTime;
    }
    public double getParkThreeshold() {
        return parkThreeshold;
    }


}
