package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Util.Globals.Alliance;
import org.firstinspires.ftc.teamcode.Util.Info;

@Config
public class Sensors {
    private Robot robot;

    Pose pose;
    double currentX,currentY,currentHeading;

    private double currentVelocityShooter = 0;
    private double voltage;
    private LynxModule controlHub, expansionHub;

    public long readVoltageTime = 0;
    private long lastUpdateTimeNs = 0;

    public double targetX = -66.6;
    public double targetY = -65;
    public double targetXRed = -70;
    public double targetYRed = 65;
    public double targetXBlue= -70;
    public double targetYBlue = -65;

    public static double STILL_MAX_TRANSLATIONAL_SPEED = 0.5; // field units per second
    public static double STILL_MAX_ANGULAR_SPEED = 1; //radians per seconds

    // Last pose used for stillness detection
    private double lastStillX = Double.NaN;
    private double lastStillY = Double.NaN;
    private double lastStillHeading = Double.NaN;
    private long lastStillCheckLoopTimeNs = 0; // Robot.loopTime at last history update

    public static double FARZONE_X1 = -72, FARZONE_Y1 = 72;
    public static double FARZONE_X2 = 0, FARZONE_Y2 = 0;
    public static double FARZONE_X3 = -72, FARZONE_Y3 = -72;

    public Sensors(Robot robot) {
        this.robot = robot;
        initSensors();
        if(Info.alliance == Alliance.BLUE){
            targetX = targetXBlue;
            targetY = targetYBlue;
        }else {
            targetX = targetXRed;
            targetY = targetYRed;
        }
    }

    private void initSensors() {
        controlHub = robot.hw.get(LynxModule.class, "Control Hub");
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        expansionHub = robot.hw.get(LynxModule.class, "Expansion Hub 2");
        expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        voltage = robot.hw.voltageSensor.iterator().next().getVoltage();
        readVoltageTime = System.currentTimeMillis();
        lastUpdateTimeNs = System.nanoTime();
    }
    public double getTargetX(){
        return targetX;
    }
    public double getTargetY(){
        return targetY;
    }
    public void update() {
        pose = robot.drive.getPose();
        currentX = pose.getX();
        currentY = pose.getY();
        currentHeading = pose.getHeading();

        if(System.currentTimeMillis() - readVoltageTime > 250) {
            voltage = robot.hw.voltageSensor.iterator().next().getVoltage();
            readVoltageTime = System.currentTimeMillis();
        }

        lastUpdateTimeNs = System.nanoTime();
    }

    public double getVoltage() {
        return voltage;
    }

    public Pose getPose() {
        return pose;
    }
    public double getX() {
        return currentX;
    }

    public double getDistanceToTarget(double targetX,double targetY){
        return Math.hypot(targetX-currentX,targetY-currentY);
    }
    public double getAngleToTarget(double targetX,double targetY) {
        return Math.atan2(targetY-currentY,targetX-currentX);
    }

    public double getAngularVelocity() {
        return robot.drive.getAngularVelocity();
    }

    public double getY() {
        return currentY;
    }
    public double getHeading() {
        return currentHeading;
    }
    public double getVelocity() {
        return currentVelocityShooter;
    }

    /**
     * Returns true if the robot is considered "still enough".
     * Safe to call multiple times per loop *after* robot.update().
     */
    public boolean isRobotStill() {
        if (pose == null) return false;

        long loopTimeNs = robot.getLoopTimeNs();

        // First-time initialization of history
        if (Double.isNaN(lastStillX) || lastStillCheckLoopTimeNs == 0) {
            lastStillX = currentX;
            lastStillY = currentY;
            lastStillHeading = currentHeading;
            lastStillCheckLoopTimeNs = loopTimeNs;
            return false; // not enough info yet
        }

        // If we've already processed this loop, just reuse the previous result
        if (loopTimeNs == lastStillCheckLoopTimeNs) {
            double dx = currentX - lastStillX;
            double dy = currentY - lastStillY;
            double dHeading = currentHeading - lastStillHeading;
            double dt = (loopTimeNs - lastStillCheckLoopTimeNs) / 1e9;
            if (dt <= 0) return false;
            double translationalSpeed = Math.hypot(dx, dy) / dt;
            double angularSpeed = Math.abs(dHeading) / dt;
            return translationalSpeed <= STILL_MAX_TRANSLATIONAL_SPEED &&
                   angularSpeed <= STILL_MAX_ANGULAR_SPEED;
        }

        double dt = (loopTimeNs - lastStillCheckLoopTimeNs) / 1e9;
        if (dt <= 0) return false;

        double dx = currentX - lastStillX;
        double dy = currentY - lastStillY;
        double dHeading = currentHeading - lastStillHeading;

        double translationalSpeed = Math.hypot(dx, dy) / dt;
        double angularSpeed = Math.abs(dHeading) / dt;

        // Update history for next loop
        lastStillX = currentX;
        lastStillY = currentY;
        lastStillHeading = currentHeading;
        lastStillCheckLoopTimeNs = loopTimeNs;

        return translationalSpeed <= STILL_MAX_TRANSLATIONAL_SPEED &&
               angularSpeed <= STILL_MAX_ANGULAR_SPEED;
    }

    public boolean isInTargetZone(double x, double y) {
        double x1 = FARZONE_X1, y1 = FARZONE_Y1;
        double x2 = FARZONE_X2, y2 = FARZONE_Y2;
        double x3 = FARZONE_X3, y3 = FARZONE_Y3;

        double denom = (y2 - y3) * (x1 - x3) + (x3 - x2) * (y1 - y3);
        if (denom == 0) {
            return false;
        }

        double a = ((y2 - y3) * (x - x3) + (x3 - x2) * (y - y3)) / denom;
        double b = ((y3 - y1) * (x - x3) + (x1 - x3) * (y - y3)) / denom;
        double c = 1 - a - b;

        double eps = 1e-6;
        return a >= -eps && b >= -eps && c >= -eps;
    }
}

