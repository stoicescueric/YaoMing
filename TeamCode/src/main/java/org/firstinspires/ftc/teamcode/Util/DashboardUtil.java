package org.firstinspires.ftc.teamcode.Util;//package Util;
//
//import com.acmerobotics.dashboard.canvas.Canvas;
//
//import org.firstinspires.ftc.teamcode.subsystems.drive.Spline;
//
//import java.util.List;
//
///**
// * Set of helper functions for drawing Road Runner paths and trajectories on dashboard canvases.
// */
//public class DashboardUtil {
//    private static final double ROBOT_RADIUS = 7; // in
//    private static final double POLE_RADIUS = 0.5; // in
//
//    public static void drawField() {
//        final int[] mirror = {1, -1};
//        Canvas canvas = TelemetryUtil.packet.fieldOverlay();
//        canvas.setFill("#e0e0e0e0");
//        canvas.fillRect(-72, -72, 144, 144); // background
//        canvas.setStroke("#808080");
//        canvas.strokeRect(-15, -24, 30, 48); // submersible
//        for (int i : mirror) {
//            for (int j : mirror) canvas.strokeLine(i * 24, j * 24, i * 36, 0); // ascent zone
//            canvas.strokeLine(i * -24, i *  24, i *  24, i *  24);
//        }
//        canvas.setStroke("#0000ff");
//        for (int i : mirror) {
//            canvas.strokeLine(i *  48, i *  72, i *  72, i *  48); // net zone
//            canvas.strokeLine(i * -72, i *  60, i * -48, i *  60); // observation zone
//            canvas.strokeLine(i * -48, i *  60, i * -36, i *  72); // observation zone
//            canvas.strokeLine(i * -15, i *  24, i *  15, i *  24); // specimen rung
//            for (int j : mirror) {
//                for (int k = 0; k < 3; ++k) {
//                    canvas.strokeLine(i * j * (-49.5 - 10 * k), i * 24, i * j * (-49.5 - 10 * k), i * 27.5); // preset
//                }
//                canvas.setStroke("#ffff00");
//            }
//            canvas.setStroke("#ff0000");
//        }
//        canvas.drawGrid(0, 0, 144, 144, 7, 7);
//    }
//
//    public static void drawPoseHistory(Canvas canvas, List<Pose2d> poseHistory) {
//        double[] xPoints = new double[poseHistory.size()];
//        double[] yPoints = new double[poseHistory.size()];
//        for (int i = 0; i < poseHistory.size(); i++) {
//            Pose2d pose = poseHistory.get(i);
//            xPoints[i] = pose.getX();
//            yPoints[i] = pose.getY();
//        }
//        canvas.strokePolyline(xPoints, yPoints);
//    }
//
//    public static void drawSampledPath(Canvas canvas, Spline spline) {
//        // JANK
//        if (spline == null) {
//            return;
//        }
//
//        int samples = spline.poses.size();
//        double[] xPoints = new double[samples];
//        double[] yPoints = new double[samples];
//        for (int i = 0; i < samples; i++) {
//            Pose2d pose = spline.poses.get(i);
//            xPoints[i] = pose.getX();
//            yPoints[i] = pose.getY();
//        }
//        canvas.setStroke("#000000");
//        canvas.strokePolyline(xPoints, yPoints);
//        canvas.strokeCircle(spline.getLastPoint().x, spline.getLastPoint().y, 6);
//    }
//
//    public static void drawRobot(Canvas canvas, Pose2d pose, String color) {
//        canvas.setStroke(color);
//        canvas.strokeCircle(pose.getX(), pose.getY(), ROBOT_RADIUS);
//        canvas.strokeCircle(pose.getX(), pose.getY(), 0.5);
//        Pose2d v = new Pose2d(Math.cos(pose.heading)*ROBOT_RADIUS, Math.sin(pose.heading)*ROBOT_RADIUS);
//        double x1 = pose.getX() + v.getX() / 2, y1 = pose.getY() + v.getY() / 2;
//        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
//        canvas.strokeLine(x1, y1, x2, y2);
//    }
//
//    public static void drawRobot(Canvas canvas, Pose2d pose, double ext, String color) {
//        canvas.setStroke(color);
//        double x = pose.getX(), y = pose.getY();
//        double s = Math.sin(pose.heading), c = Math.cos(pose.heading);
//        double re1 = 6, re2 = 9 + ext, rh = 4;
//        double x1 = x + c * re1, y1 = y + s * re1;
//        double x2 = x + c * re2, y2 = y + s * re2;
//        double xh = x + c * rh, yh = y + s * rh;
//        canvas.strokeCircle(x, y, ROBOT_RADIUS);
//        canvas.strokeCircle(x, y, 0.5);
//        canvas.strokeLine(x, y, xh, yh);
//        canvas.strokeLine(x1, y1, x2, y2);
//        canvas.strokeCircle(x2, y2, 0.5);
//    }
//
//    public static void drawPole(Canvas canvas, Pose2d pose) {
//        canvas.strokeCircle(pose.getX(), pose.getY(), POLE_RADIUS);
//    }
//}