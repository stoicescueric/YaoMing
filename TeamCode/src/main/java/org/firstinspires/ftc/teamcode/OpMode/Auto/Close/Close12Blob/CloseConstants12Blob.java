package org.firstinspires.ftc.teamcode.OpMode.Auto.Close.Close12Blob;


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
public class CloseConstants12Blob {

    //DEFAULT VALUES FOR RED
    public static double shootingTime = 1350;
    public static double turretPositionRed = 0.598;
    public static double turretPositionBlue = 0.39;
    public static double failSafeDtTime = 3500;
    public static double hoodPosition = 0.615;
    public static double launcherVelocity = 1000;
    public static double startX = -62.22, startY = 37.00, headingStartRed =  Math.toRadians(0);
    public Pose startPose;
    public static double shootingX = -4.1, shootingY = 14.5, shootingHeading = Math.toRadians(270);
    public Pose scorePose;
    public static double pickUp1X = -11.3, pickUp1Y = 51, pickUp1Heading = Math.toRadians(270);
    public Pose pickUpPose;
    public static double max_power_pickUp = 0.85;
    public static double max_power_clear = 0.54;
    public static double pickUp2XIntermediary = 13.38, pickUp2YIntermediary = 30, pickUp2HeadingIntermediary = Math.toRadians(270);

    public static double pickUp2X = 13.6, pickUp2Y = 50.5, pickUp2Heading = Math.toRadians(270);
    public Pose pickUpPose2;
    public Pose pickUpPose2Intermediary;

    public static double pickUp3X = 36.75, pickUp3Y = 50.5, pickUp3Heading = Math.toRadians(270);
    public Pose pickUpPose3;
    public static double pickUp3XIntermediary = 36.75, pickUp3YIntermediary = 29.52, pickUp3HeadingIntermediary = Math.toRadians(270);
    public Pose pickUpPose3Intermediary;

    //clear
    public static double clearX = 1, clearY = 54.7, clearHeading = Math.toRadians(270);
    public static double clearXInter = 2.79, clearYInter = 46.8, clearHeadingInter = Math.toRadians(270);
    public Pose clear;
    public Pose clearInter;

    public static double parkX = -3.5, parkY = 24.7, parkHeading = Math.toRadians(270);

    public Pose parkPose;

    public void buildPaths(Follower follower) {

        parkPose = new Pose(parkX, parkY * (Info.alliance == Alliance.RED ? 1 : -1), parkHeading * (Info.alliance == Alliance.RED ? 1 : -1));
        startPose = new Pose(startX, startY * (Info.alliance == Alliance.RED ? 1 : -1),headingStartRed * (Info.alliance == Alliance.RED ? 1 : -1) );
        scorePose = new Pose(shootingX, shootingY * (Info.alliance == Alliance.RED ? 1 : -1), shootingHeading * (Info.alliance == Alliance.RED ? 1 : -1));

        pickUpPose = new Pose(pickUp1X, pickUp1Y * (Info.alliance == Alliance.RED ? 1 : -1), pickUp1Heading * (Info.alliance == Alliance.RED ? 1 : -1));
        pickUpPose2 = new Pose(pickUp2X, pickUp2Y * (Info.alliance == Alliance.RED ? 1 : -1), pickUp2Heading * (Info.alliance == Alliance.RED ? 1 : -1));
        pickUpPose3 = new Pose(pickUp3X, pickUp3Y * (Info.alliance == Alliance.RED ? 1 : -1), pickUp3Heading * (Info.alliance == Alliance.RED ? 1 : -1));
        clear = new Pose(clearX, clearY * (Info.alliance == Alliance.RED ? 1 : -1), clearHeading * (Info.alliance == Alliance.RED ? 1 : -1));
        clearInter = new Pose(clearXInter, clearYInter * (Info.alliance == Alliance.RED ? 1 : -1), clearHeadingInter * (Info.alliance == Alliance.RED ? 1 : -1));

        pickUpPose2Intermediary = new Pose(pickUp2XIntermediary, pickUp2YIntermediary, pickUp2HeadingIntermediary);
        pickUpPose3Intermediary = new Pose(pickUp3XIntermediary, pickUp3YIntermediary, pickUp3HeadingIntermediary);
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
    public double getMaxClearPower() {return max_power_clear;}
    public double getFailSafeDtTime() {
        return failSafeDtTime;
    }


}
