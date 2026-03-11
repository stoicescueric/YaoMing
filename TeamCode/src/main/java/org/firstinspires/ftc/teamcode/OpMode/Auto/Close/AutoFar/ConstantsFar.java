package org.firstinspires.ftc.teamcode.OpMode.Auto.Close.AutoFar;

import com.pedropathing.geometry.Pose;

public class ConstantsFar {
    public static double startX = 0,startY = 0,startHeading = Math.toRadians(0);

    public static double shootX = 0,shootY = 0,shootHeading = Math.toRadians(0);

    public static double preloadX = 0,preloadY = 0,preloadHeading = Math.toRadians(0);

    public static double stacX_1 = 0,stackY_1 = 0,stackHeading_1 = Math.toRadians(0);
    public static double stackX_2 = 0,stackY_2 = 0,stackHeading_2 = Math.toRadians(0);


    Pose startPose;
    Pose shootingPose;
    Pose preload;
    public static double hp_runs = 3;
    public static double shootingTime = 1000;
    public static double dtFailsafe = 2000;
    Pose stackPose1;
    Pose stackPose2;

    public void buildPath() {
            startPose = new Pose(startX,startY,startHeading);
            shootingPose = new Pose(shootX,shootY,shootHeading);
            preload = new Pose(preloadX,preloadY,preloadHeading);
            stackPose1 = new Pose(stacX_1,stackY_1,stackHeading_1);
            stackPose2 = new Pose(stackX_2,stackY_2,stackHeading_2);
    }

    public double getDtFailsafe() {
        return dtFailsafe;
    }
    public double getShootingTime() {
        return shootingTime;
    }

}
