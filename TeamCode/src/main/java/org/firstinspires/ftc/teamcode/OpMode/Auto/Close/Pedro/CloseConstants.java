package org.firstinspires.ftc.teamcode.OpMode.Auto.Close.Pedro;


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
public class CloseConstants {

    //DEFAULT VALUES FOR RED
    public static double shootingTime = 1500;
    public static double turretPositionRed = 0.598;
    public static double turretPositionBlue = 0.39;
    public static double failSafeDtTime = 3500;
    public static double hoodPosition = 0.615;
    public static double launcherVelocity = 1000;
    public static double startX = -58.72, startY = 44.74, headingStartRed =  -0.87598; //TODO
    public Pose startPose;
    public static double shootingX = -14, shootingY = 12, shootingHeading = Math.toRadians(90);
    public Pose scorePose;

    public static double pickUp1X = -11.3, pickUp1Y = 52, pickUp1Heading = Math.toRadians(90);
    public Pose pickUpPose;
    public static double max_power_pickUp = 0.85;
    public static double max_power_clear = 0.54;
    public static double pickUp2XIntermediary = 17, pickUp2YIntermediary = 8, pickUp2HeadingIntermediary = Math.toRadians(90);

    public static double pickUp2X = 14.5, pickUp2Y = 59, pickUp2Heading = Math.toRadians(90);
    public Pose pickUpPose2;
    public Pose pickUpPose2Intermediary;

    public static double pickUp3X = 38, pickUp3Y = 59, pickUp3Heading = Math.toRadians(90);
    public Pose pickUpPose3;
    public static double pickUp3XIntermediary = 44, pickUp3YIntermediary = -5, pickUp3HeadingIntermediary = Math.toRadians(90);
    public Pose pickUpPose3Intermediary;

    //clear
    public static double clearX = 0.5, clearY = 55, clearHeading = Math.toRadians(90);
    public Pose clear;
    public static double clearXIntermediary = 0.5, clearYIntermediary = 40, clearHeadingIntermediary = Math.toRadians(90);
    public Pose clearIntermediary;

    public static double parkX = 13.5, parkY = 45, parkHeading = Math.toRadians(90);

    public Pose parkPose;

    Path scorePreload;
    PathChain grabPickUp1, scorePickup1, grabPickup2, scorePickup2;
    PathChain grabPickup3, scorePickup3;
    PathChain goClear;
    PathChain goToPark;

    public void buildPaths(Follower follower) {

        parkPose = new Pose(parkX, parkY * (Info.alliance == Alliance.RED ? 1 : -1), parkHeading * (Info.alliance == Alliance.RED ? 1 : -1));
        startPose = new Pose(startX, startY * (Info.alliance == Alliance.RED ? 1 : -1),headingStartRed * (Info.alliance == Alliance.RED ? 1 : -1) );
        scorePose = new Pose(shootingX, shootingY * (Info.alliance == Alliance.RED ? 1 : -1), shootingHeading * (Info.alliance == Alliance.RED ? 1 : -1));

        pickUpPose = new Pose(pickUp1X, pickUp1Y * (Info.alliance == Alliance.RED ? 1 : -1), pickUp1Heading * (Info.alliance == Alliance.RED ? 1 : -1));
        pickUpPose2 = new Pose(pickUp2X, pickUp2Y * (Info.alliance == Alliance.RED ? 1 : -1), pickUp2Heading * (Info.alliance == Alliance.RED ? 1 : -1));
        pickUpPose3 = new Pose(pickUp3X, pickUp3Y * (Info.alliance == Alliance.RED ? 1 : -1), pickUp3Heading * (Info.alliance == Alliance.RED ? 1 : -1));
        clear = new Pose(clearX, clearY * (Info.alliance == Alliance.RED ? 1 : -1), clearHeading * (Info.alliance == Alliance.RED ? 1 : -1));

        pickUpPose2Intermediary = new Pose(pickUp2XIntermediary, pickUp2YIntermediary, pickUp2HeadingIntermediary);
        pickUpPose3Intermediary = new Pose(pickUp3XIntermediary, pickUp3YIntermediary, pickUp3HeadingIntermediary);
        clearIntermediary = new Pose(clearXIntermediary, clearYIntermediary * (Info.alliance == Alliance.RED ? 1 : -1), clearHeadingIntermediary * (Info.alliance == Alliance.RED ? 1 : -1));

        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPickUp1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickUpPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickUpPose.getHeading())
                .build();
        goClear = follower.pathBuilder()
                .addPath(new BezierCurve(pickUpPose, clearIntermediary, clear))
                .setLinearHeadingInterpolation(pickUpPose3.getHeading(), clear.getHeading())
                .build();;
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(clear, scorePose))
                .setLinearHeadingInterpolation(pickUpPose.getHeading(), scorePose.getHeading())
                .build();
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, pickUpPose2Intermediary, pickUpPose2))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickUpPose2.getHeading())
                .build();
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickUpPose2, pickUpPose2Intermediary, scorePose))
                .setLinearHeadingInterpolation(pickUpPose2.getHeading(), scorePose.getHeading())
                .build();
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, pickUpPose3Intermediary, pickUpPose3))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickUpPose3.getHeading())
                .build();
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(pickUpPose3, pickUpPose3Intermediary, scorePose))
                .setLinearHeadingInterpolation(pickUpPose3.getHeading(), scorePose.getHeading())
                .build();

        goToPark = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();

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
