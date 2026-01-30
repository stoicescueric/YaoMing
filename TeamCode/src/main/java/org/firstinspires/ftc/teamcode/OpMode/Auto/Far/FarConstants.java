package org.firstinspires.ftc.teamcode.OpMode.Auto.Far;

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

    public static double shootingTime = 3350;
    public static double failSafeDtTime = 2500;
    public static double startX = 63.17, startY = 18.30, headingStartRed = Math.PI;
    public Pose startPose;
    public static double shootingX = 50, shootingY = 8.5, shootingHeading = Math.PI;
    public Pose scorePose;
    public static double pickUp1X = 36, pickUp1Y = 57, pickUp1Heading = Math.PI/2;
    public static double pickUp1XIntermediary = 26, pickUp1YIntermediary = 20, pickUp1HeadingIntermediary = Math.PI/2;
    public Pose pickUpPose;
    public Pose pickUpPose1Intermediary;
    public static double pickUp2XIntermediary = 3, pickUp2YIntermediary = 18, pickUp2HeadingIntermediary = Math.PI/2;
    public static double pickUp2X = 10.5, pickUp2Y = 57.5  , pickUp2Heading = Math.PI/2;
    public Pose pickUpPose2;
    public Pose pickUpPose2Intermediary;
    public static double parkX = 60, parkY = 34, parkHeading = Math.PI/2;
    public Pose parkPose;
    public static double max_power_pickUp = 0.82;
    public static double max_power_return = 0.82;
    public Path scorePreload;
    public PathChain grabPickUp1, scorePickup1, grabPickup2, scorePickup2;
    public PathChain goToPark;

    public void buildPaths(Follower follower) {
        parkPose = new Pose(parkX, parkY * (Info.alliance == Alliance.RED ? 1 : -1), parkHeading * (Info.alliance == Alliance.RED ? 1 : -1));
        startPose = new Pose(startX, startY * (Info.alliance == Alliance.RED ? 1 : -1), headingStartRed * (Info.alliance == Alliance.RED ? 1 : -1));
        scorePose = new Pose(shootingX, shootingY * (Info.alliance == Alliance.RED ? 1 : -1), shootingHeading * (Info.alliance == Alliance.RED ? 1 : -1));

        pickUpPose = new Pose(pickUp1X, pickUp1Y * (Info.alliance == Alliance.RED ? 1 : -1), pickUp1Heading * (Info.alliance == Alliance.RED ? 1 : -1));
        pickUpPose2 = new Pose(pickUp2X, pickUp2Y * (Info.alliance == Alliance.RED ? 1 : -1), pickUp2Heading * (Info.alliance == Alliance.RED ? 1 : -1));

        pickUpPose2Intermediary = new Pose(pickUp2XIntermediary, pickUp2YIntermediary * (Info.alliance == Alliance.RED ? 1 : -1), pickUp2HeadingIntermediary * (Info.alliance == Alliance.RED ? 1 : -1));
        pickUpPose1Intermediary = new Pose(pickUp1XIntermediary, pickUp1YIntermediary * (Info.alliance == Alliance.RED ? 1 : -1), pickUp1HeadingIntermediary * (Info.alliance == Alliance.RED ? 1 : -1));

        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        Path grabPickUp1Path = new Path(new BezierCurve(scorePose, pickUpPose1Intermediary, pickUpPose));
        grabPickUp1Path.setLinearHeadingInterpolation(
                scorePose.getHeading(),
                pickUpPose.getHeading(),
                0.3
        );

        grabPickUp1 = follower.pathBuilder()
                .addPath(grabPickUp1Path)
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickUpPose, scorePose))
                .setLinearHeadingInterpolation(pickUpPose.getHeading(), scorePose.getHeading())
                .build();

        Path grabPickup2Path = new Path(new BezierCurve(scorePose, pickUpPose2Intermediary, pickUpPose2));
        grabPickup2Path.setLinearHeadingInterpolation(
                scorePose.getHeading(),
                pickUpPose2.getHeading(),
                0.3
        );

        grabPickup2 = follower.pathBuilder()
                .addPath(grabPickup2Path)
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickUpPose2, scorePose))
                .setLinearHeadingInterpolation(pickUpPose2.getHeading(), scorePose.getHeading())
                .build();

        goToPark = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
    }

    public double getShootingTime() { return shootingTime; }
    public double getMaxPower(){ return max_power_pickUp; }
    public double getMaxReturnPower(){return max_power_return;}
    public double getFailSafeDtTime() { return failSafeDtTime; }
}
