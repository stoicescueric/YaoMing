package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Util.Globals.Alliance;
import org.firstinspires.ftc.teamcode.Util.Info;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.OuttakePositions;

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

    private boolean intakeMotor1OverCurrent = false;
    private boolean intakeSensorHigh = false;

    public double targetX = -66.6;
    public double targetY = -65;
    public double targetXRedClose = -69.8;
    public double targetYRedClose = 69.8;
    public double targetXBlueClose = -69.5;
    public double targetYBlueClose = -69.5;

    public double targetXRedFar = -71;
    public double targetYRedFar = 70;
    public double targetXBlueFar = -71;
    public double targetYBlueFar = -72;

    public double backboardX;
    public double backboardY;
    public double backboardXRed = -72;
    public double backboardYRed =72;
    public double backboardXBlue = -73;
    public double backboardYBlue = -73;
    public double intakeSpeed;

    public static double STILL_MAX_TRANSLATIONAL_SPEED = 13; // field units per second
    public static double STILL_MAX_ANGULAR_SPEED = 12; //radians per seconds
    private double lastStillX = Double.NaN;
    private double lastStillY = Double.NaN;
    private double lastStillHeading = Double.NaN;
    private long lastStillCheckLoopTimeNs = 0; // Robot.loopTime at last history update

    public static double CLOSEZONE_X1 = -89, CLOSEZONE_Y1 = 89;
    public static double CLOSEZONE_X2 = 15,  CLOSEZONE_Y2 = 0;
    public static double CLOSEZONE_X3 = -89, CLOSEZONE_Y3 = -89;
    public static double FARZONE_X1 = 63, FARZONE_Y1 = 24;
    public static double FARZONE_X2 = 44,  FARZONE_Y2 = 0;
    public static double FARZONE_X3 = 63,  FARZONE_Y3 = -24;



    public static double SLOWZONE_X1 = 10, SLOWZONE_Y1 = 35;
    public static double SLOWZONE_X2 = 35, SLOWZONE_Y2 = 35;
    public static double SLOWZONE_X3 = 10, SLOWZONE_Y3 = 72;
    public static double SLOWZONE_X4 = 10, SLOWZONE_Y4 = 72;

    private double velX = 0.0;
    private double velY = 0.0;

    public static double DEFAULT_PROJECTILE_SPEED = 300.0; // inches per second, rough guess

    private DigitalChannel intakeColor;

    public Sensors(Robot robot) {
        this.robot = robot;
        initSensors();
        if(Info.alliance == Alliance.BLUE){
            targetX = targetXBlueClose;
            targetY = targetYBlueClose;
            backboardX = backboardXBlue;
            backboardY = backboardYBlue;
        }else {
            targetX = targetXRedClose;
            targetY = targetYRedClose;
            backboardX = backboardXRed;
            backboardY = backboardYRed;
        }
    }

    private void initSensors() {
        controlHub = robot.hw.get(LynxModule.class, "Control Hub");
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        expansionHub = robot.hw.get(LynxModule.class, "Expansion Hub 2");
        expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        intakeColor = robot.hw.get(DigitalChannel.class, "intakeColor");
        intakeColor.setMode(DigitalChannel.Mode.INPUT);

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
    public double getBackboardX() { return backboardX; }
    public double getBackboardY() { return backboardY; }

    public void update() {
        Pose prevPose = pose;
        double prevX = currentX;
        double prevY = currentY;

        long prevUpdateTimeNs = lastUpdateTimeNs;

        pose = robot.drive.getPose();
        currentX = pose.getX();
        currentY = pose.getY();
        currentHeading = pose.getHeading();

        long nowNs = System.nanoTime();
        if (prevUpdateTimeNs != 0) {
            double dt = (nowNs - prevUpdateTimeNs) / 1e9; // seconds
            if (dt > 1e-4) {
                velX = (currentX - prevX) / dt;
                velY = (currentY - prevY) / dt;
            }
        }

        if(System.currentTimeMillis() - readVoltageTime > 250) {
            voltage = robot.hw.voltageSensor.iterator().next().getVoltage();
            readVoltageTime = System.currentTimeMillis();
        }

        if (robot.intakeTransfer != null && robot.intakeTransfer.motor1 != null) {
            intakeMotor1OverCurrent = robot.intakeTransfer.motor1.isOverCurrent();
            intakeSpeed = robot.intakeTransfer.motor1.getVelocity();
        } else {
            intakeMotor1OverCurrent = false;
        }

        if (intakeColor != null) {
            intakeSensorHigh = (intakeColor.getState());
        } else {
            intakeSensorHigh = false;
        }

        lastUpdateTimeNs = nowNs;
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

    public double getDistanceToBackboard() {
        return Math.hypot(backboardX - currentX, backboardY - currentY);
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
    public double getVelocityIntake() {
        return intakeSpeed;
    }
    public double getHeading() {
        return currentHeading;
    }
    public double getVelocity() {
        return currentVelocityShooter;
    }

    /**
     * Returns true if the intake conveyor motor1 is currently over its current limit.
     * This value is updated once per Sensors.update() call using the hub bulk data.
     */
    public boolean isIntakeMotor1OverCurrent() {
        return intakeMotor1OverCurrent;
    }

    /**
     * Returns true if the robot is considered in the long/far shooting zone
     * based purely on its X coordinate vs FAR_ZONE_X_THRESHOLD.
     * This is the single source of truth for "long shot" checks.
     */
    public boolean shootingLong() {
        return currentX > OuttakePositions.FAR_ZONE_X_THRESHOLD;
    }

    /**
     * Returns true if the robot is considered "still enough".
     * Safe to call multiple times per loop *after* robot.update().
     */
    public boolean isRobotStill() {
        if (pose == null) return false;

        long loopTimeNs = robot.getLoopTimeNs();

        if (Double.isNaN(lastStillX) || lastStillCheckLoopTimeNs == 0) {
            lastStillX = currentX;
            lastStillY = currentY;
            lastStillHeading = currentHeading;
            lastStillCheckLoopTimeNs = loopTimeNs;
            return false;
        }
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

        lastStillX = currentX;
        lastStillY = currentY;
        lastStillHeading = currentHeading;
        lastStillCheckLoopTimeNs = loopTimeNs;

        return translationalSpeed <= STILL_MAX_TRANSLATIONAL_SPEED &&
               angularSpeed <= STILL_MAX_ANGULAR_SPEED;
    }

    public boolean isInTargetZone(double x, double y) {
        if (isPointInTriangle(x, y,
                CLOSEZONE_X1, CLOSEZONE_Y1,
                CLOSEZONE_X2, CLOSEZONE_Y2,
                CLOSEZONE_X3, CLOSEZONE_Y3)) {
            return true;
        }

        if (isPointInTriangle(x, y,
                FARZONE_X1, FARZONE_Y1,
                FARZONE_X2, FARZONE_Y2,
                FARZONE_X3, FARZONE_Y3)) {
            return true;
        }

        return false;
    }
    private boolean isPointInTriangle(double x, double y,
                                      double x1, double y1,
                                      double x2, double y2,
                                      double x3, double y3) {
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

    public void updateTargetForZone() {
        boolean isFar = currentX > OuttakePositions.FAR_ZONE_X_THRESHOLD;

        if (Info.alliance == Alliance.BLUE) {
            targetX = isFar ? targetXBlueFar : targetXBlueClose;
            targetY = isFar ? targetYBlueFar : targetYBlueClose;
        } else {
            targetX = isFar ? targetXRedFar : targetXRedClose;
            targetY = isFar ? targetYRedFar : targetYRedClose;
        }
    }

    /**
     * Returns the current estimated translational velocity of the robot in the
     * field frame (units per second).
     */
    public double getVelX() { return velX; }
    public double getVelY() { return velY; }

    /**
     * Approximate time of flight for a shot given a planar distance. This is a
     * rough estimate using a single projectile speed parameter and can be
     * tuned from the dashboard. Units: seconds.
     */
    public double estimateShotTimeOfFlight(double distance) {
        double speed = DEFAULT_PROJECTILE_SPEED;
        if (speed <= 1e-3) return 0.0;
        return Math.max(0.0, distance / speed);
    }

    /**
     * Returns true if the robot is currently inside the slow zone
    **/
    public boolean isInSlowZone() {
        double[] xs = { SLOWZONE_X1, SLOWZONE_X2, SLOWZONE_X3, SLOWZONE_X4 };
        double[] ys;

        if (Info.alliance == Alliance.BLUE) {
            ys = new double[]{ -SLOWZONE_Y1, -SLOWZONE_Y2, -SLOWZONE_Y3, -SLOWZONE_Y4 };
        } else {
            ys = new double[]{ SLOWZONE_Y1, SLOWZONE_Y2, SLOWZONE_Y3, SLOWZONE_Y4 };
        }

        return isPointInPolygon(currentX, currentY, xs, ys);
    }

    /**
     * Generic point-in-polygon test (ray casting). xs/ys are vertices in order.
     */
    private boolean isPointInPolygon(double x, double y, double[] xs, double[] ys) {
        boolean inside = false;
        int n = xs.length;
        if (n < 3) return false;

        for (int i = 0, j = n - 1; i < n; j = i++) {
            double xi = xs[i], yi = ys[i];
            double xj = xs[j], yj = ys[j];

            boolean intersect = ((yi > y) != (yj > y)) &&
                    (x < (xj - xi) * (y - yi) / ((yj - yi) + 1e-9) + xi);
            if (intersect) inside = !inside;
        }
        return inside;
    }

    /**
     * Returns true if the intake color digital sensor is reporting HIGH.
     * If the sensor is not configured or unavailable, this returns false.
     */
    public boolean intakeSensorHigh() {
        return intakeSensorHigh;
    }
}
