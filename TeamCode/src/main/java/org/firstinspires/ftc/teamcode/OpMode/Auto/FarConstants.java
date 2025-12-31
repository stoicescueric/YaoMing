package org.firstinspires.ftc.teamcode.OpMode.Auto;

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
public class FarConstants {

    public static double shootingTime = 2850;
    public static double failSafeDtTime = 2500;
    public static double startX = 70, startY = 18.13, headingStartRed = Math.PI;
    public Pose startPose;
    public static double shootingX = 70, shootingY = 18.3, shootingHeading = Math.PI/2;
    public Pose scorePose;
    public static double pickUp1X = 34, pickUp1Y = 61, pickUp1Heading = Math.PI/2;
    public static double pickUp1XIntermediary = 34, pickUp1YIntermediary = 27, pickUp1HeadingIntermediary = Math.PI/2;
    public Pose pickUpPose;
    public Pose pickUpPose1Intermediary;
    public static double pickUp2XIntermediary = 10.8, pickUp2YIntermediary = 24, pickUp2HeadingIntermediary = Math.PI/2;
    public static double pickUp2X = 10, pickUp2Y = 61, pickUp2Heading = Math.PI/2;
    public Pose pickUpPose2;
    public Pose pickUpPose2Intermediary;
    public static double parkX = 60, parkY = 34, parkHeading = Math.PI/2;
    public Pose parkPose;
    public static double max_power_pickUp = 0.8;
    public Path scorePreload;
    public PathChain grabPickUp1, scorePickup1, grabPickup2, scorePickup2;
    public PathChain goToPark;

    public void buildPaths(Follower follower) {
        parkPose = new Pose(parkX, parkY * (Info.alliance == Alliance.RED ? 1 : -1), parkHeading * (Info.alliance == Alliance.RED ? 1 : -1));
        startPose = new Pose(startX, startY * (Info.alliance == Alliance.RED ? 1 : -1), headingStartRed * (Info.alliance == Alliance.RED ? 1 : -1));
        scorePose = new Pose(shootingX, shootingY * (Info.alliance == Alliance.RED ? 1 : -1), shootingHeading * (Info.alliance == Alliance.RED ? 1 : -1));

        pickUpPose = new Pose(pickUp1X, pickUp1Y * (Info.alliance == Alliance.RED ? 1 : -1), pickUp1Heading * (Info.alliance == Alliance.RED ? 1 : -1));
        pickUpPose2 = new Pose(pickUp2X, pickUp2Y * (Info.alliance == Alliance.RED ? 1 : -1), pickUp2Heading * (Info.alliance == Alliance.RED ? 1 : -1));

        pickUpPose2Intermediary = new Pose(pickUp2XIntermediary, pickUp2YIntermediary, pickUp2HeadingIntermediary);
        pickUpPose1Intermediary = new Pose(pickUp1XIntermediary, pickUp1YIntermediary, pickUp1HeadingIntermediary);

        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPickUp1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, pickUpPose1Intermediary, pickUpPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickUpPose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickUpPose, scorePose))
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

        goToPark = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
    }

    public double getShootingTime() { return shootingTime; }
    public double getMaxPower(){ return max_power_pickUp; }
    public double getFailSafeDtTime() { return failSafeDtTime; }
}
