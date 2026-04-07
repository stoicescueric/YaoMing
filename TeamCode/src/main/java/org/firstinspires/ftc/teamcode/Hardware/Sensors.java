package org.firstinspires.ftc.teamcode.Hardware;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Util.Caching.CachingServo;
import org.firstinspires.ftc.teamcode.Util.Globals.Alliance;
import org.firstinspires.ftc.teamcode.Util.Globals.Phase;
import org.firstinspires.ftc.teamcode.Util.Info;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.OuttakePositions;
import org.firstinspires.ftc.teamcode.Util.Wrapper.CachingVoltageSensor;
import org.firstinspires.ftc.teamcode.Util.Wrapper.InterpLUT;
import org.firstinspires.ftc.teamcode.blob.math.LowPassFilter;

@Config
public class Sensors {
    private Robot robot;

    CachingServo light;
    InterpLUT shotTime;

    public enum LightColor {
        OFF(0),
        RED(0.3),
        GREEN(0.5),
        BLUE(0.611);
        final double value;
        LightColor(double value) {
            this.value = value;
        }
    }
    public LightColor lightColor = LightColor.OFF;
    Pose pose;
    double currentX,currentY,currentHeading;
    public static double FORWARD_TURRET_OFFSET = -1.66;

    private double currentVelocityShooter = 0;
    private double voltage;

    public long readVoltageTime = 0;
    private long lastUpdateTimeNs = 0;

    private boolean intakeMotor1OverCurrent = false;
    private boolean breakBeamPos1High = false;
    private boolean breakBeamPos2High = false;
    private boolean breakBeamPos3High = false;

    public  double targetX = -66.6;
    public  double targetY = -65;
    public static double targetXRedClose = -71;
    public static double targetYRedClose = 67;
    public double targetXBlueClose = -71;
    public double targetYBlueClose = -67;

    public static double targetXRedFar = -72;
    public static double servoPos = 0.4;
    public static double targetYRedFar = 64;
    public double targetXBlueFar = -72;
    public double targetYBlueFar = -64;
    public double virtualTargetX = targetX;
    public double virtualTargetY = targetY;


    public double intakeSpeed;
    double shooterWorldX, shooterWorldY;

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

    public double projectedX,projectedY;

    public boolean sotm = false;

    public static double switchTarget = 25;
    public static double SLOWZONE_X1 = 10, SLOWZONE_Y1 = 35;
    public static double SLOWZONE_X2 = 35, SLOWZONE_Y2 = 35;
    public static double SLOWZONE_X3 = 10, SLOWZONE_Y3 = 72;
    public static double SLOWZONE_X4 = 10, SLOWZONE_Y4 = 72;

    private double velX = 0.0;
    private double velY = 0.0;
    public double xVelocityRobot, yVelocityRobot;
    public double xAccRobot, yAccRobot;
    public static double filterParameter = 0.65;
    private  final LowPassFilter xVelocityFilter = new LowPassFilter(filterParameter, 0);
    private final LowPassFilter yVelocityFilter = new LowPassFilter(filterParameter, 0);

    public static double DEFAULT_PROJECTILE_SPEED = 300.0; // inches per second, rough guess

    private DigitalChannel breakBeamPos1, breakBeamPos2, breakBeamPos3;
    public boolean lastValueBreakBreamPos1,lastValuebreamBeamPos2,lastValuebreamBeamPos3;

    public long firstTrueBeam1,firstTrueBeam2,firstTrueBeam3;
    public Sensors(Robot robot) {
        this.robot = robot;
        initSensors();
        if(Info.alliance == Alliance.BLUE){
            targetX = targetXBlueClose;
            targetY = targetYBlueClose;
        }else {
            targetX = targetXRedClose;
            targetY = targetYRedClose;
        }
        shotTime = new InterpLUT();
        createShotTime();
    }

    void createShotTime() {
        shotTime.add(42,0.695);
        shotTime.add(52,0.516);
        shotTime.add(62,0.704);
        shotTime.add(72,0.608);
        shotTime.add(82,0.670);
        shotTime.createLUT();
    }
    public static boolean usePredictivePose = true;
    public static double timeLatency = 0.32; //sec
    private void initSensors() {

        light = new CachingServo(robot.hw.get(Servo.class,"led"));
        voltageSensor = new CachingVoltageSensor(robot.hw);
        breakBeamPos1 = robot.hw.get(DigitalChannel.class, "beamBrakePos1");;
        breakBeamPos1.setMode(DigitalChannel.Mode.INPUT);
        lastValueBreakBreamPos1 = breakBeamPos1.getState();
        breakBeamPos2 = robot.hw.get(DigitalChannel.class, "beamBrakePos2");;
        breakBeamPos2.setMode(DigitalChannel.Mode.INPUT);
        lastValuebreamBeamPos2= breakBeamPos2.getState();
        breakBeamPos3 = robot.hw.get(DigitalChannel.class, "beamBrakePos3");;
        breakBeamPos3.setMode(DigitalChannel.Mode.INPUT);
        lastValuebreamBeamPos3= breakBeamPos3.getState();

        voltage = robot.hw.voltageSensor.iterator().next().getVoltage();
        readVoltageTime = System.currentTimeMillis();
        lastUpdateTimeNs = System.nanoTime();
    }
    public double getTargetX(){
        if(sotm) {
            return virtualTargetX;
        }
        return targetX;
    }
    public double getTargetY(){
        if(sotm) {
            return virtualTargetY;
        }
        return targetY;
    }
    CachingVoltageSensor voltageSensor;

    public double getDistanceBetweenPoints(double x1,double x2,double y1,double y2){
        return Math.sqrt((x1 - x2) * (x1-x2) + (y1 - y2) * (y1 - y2));
    }
    public static double threesholdTime = 0.050;

    public static double latencyFactor = 0.07;
    public static double projectedXSign = 1;
    public static double projectedYSign = 1;
    public boolean isFarZone() {
        return currentX > switchTarget;
    }

    public void update() {
        Pose prevPose = pose;

        double prevX = currentX;
        double prevY = currentY;

        long prevUpdateTimeNs = lastUpdateTimeNs;
        pose = robot.blob.odo.getPose();

        currentX = pose.getX();
        currentY = pose.getY();
        currentHeading = pose.getHeading();


        calculateVelAndAcc();


        projectedX = currentX + (xVelocityRobot * timeLatency * projectedXSign);
        projectedY = currentY + (yVelocityRobot * timeLatency * projectedYSign);

        if(usePredictivePose && Info.phase == Phase.TELEOP) {
            currentX = currentX + (xVelocityRobot * timeLatency);
            currentY = currentY + (yVelocityRobot * timeLatency);
        }





        shooterWorldX = currentX + (FORWARD_TURRET_OFFSET * Math.cos(currentHeading));
        shooterWorldY = currentY + (FORWARD_TURRET_OFFSET * Math.sin(currentHeading));
        double shotTimeEstimate = shotTime.get(getShooterDistanceToBackboard());
        if(isFarZone()) {
            if(Info.alliance == Alliance.RED) {
                targetX = targetXRedFar;
                targetY = targetYRedFar;
            }else {
                targetX = targetXBlueFar;
                targetY = targetYBlueFar;
            }
        }else {
            if(Info.alliance == Alliance.RED) {
                targetX = targetXRedClose;
                targetY = targetYRedClose;
            }else {
                targetX = targetXBlueClose;
                targetY = targetYBlueClose;
            }
        }

        for(int i = 0;i<5;i++) {
            virtualTargetX = targetX - shotTimeEstimate * (xVelocityRobot + latencyFactor * xAccRobot);
            virtualTargetY = targetY - shotTimeEstimate * (yVelocityRobot + latencyFactor * yAccRobot);

            double newShotTime = shotTime.get(getDistanceBetweenPoints(virtualTargetX,currentX,virtualTargetY,currentY));
            if(Math.abs(newShotTime - shotTimeEstimate) <= threesholdTime) {
                i = 4;
            }
            if(i != 4) {
                shotTimeEstimate = newShotTime;
            }
        }







        if (breakBeamPos1 != null) {
            lastValueBreakBreamPos1 = breakBeamPos1High;
            breakBeamPos1High = !(breakBeamPos1.getState());
            if(breakBeamPos1High && !lastValueBreakBreamPos1) {
                firstTrueBeam1 = System.currentTimeMillis();
            }else if(!breakBeamPos1High && !lastValueBreakBreamPos1) {
                firstTrueBeam1 = System.currentTimeMillis();
            }
        } else {
            breakBeamPos1High = false;
        }
        if (breakBeamPos2 != null) {
            lastValuebreamBeamPos2 = breakBeamPos2High;
            breakBeamPos2High = !(breakBeamPos2.getState());
            if(breakBeamPos2High && !lastValuebreamBeamPos2) {
                firstTrueBeam2 = System.currentTimeMillis();
            }else if(!breakBeamPos2High && !lastValuebreamBeamPos2) {
                firstTrueBeam2 = System.currentTimeMillis();
            }
        } else {
            breakBeamPos2High = false;
        }
        if (breakBeamPos3 != null) {
            lastValuebreamBeamPos3 = breakBeamPos3High;
            breakBeamPos3High = !(breakBeamPos3.getState());

            if(breakBeamPos3High && !lastValuebreamBeamPos3) {
                firstTrueBeam3 = System.currentTimeMillis();
            }else if(!breakBeamPos3High && !lastValuebreamBeamPos3) {
                firstTrueBeam3 = System.currentTimeMillis();
            }
        } else {
            breakBeamPos3High = false;
        }
        shooterAngle =  Math.atan2(targetY-shooterWorldY, targetX-shooterWorldX);
        calculateDistance();
       // Log.w("beam ","Beam braked 1 : " + breakBeamPos1High + "Beam braked 2 : " + breakBeamPos2High + "Beam braked 3 : " + breakBeamPos3High);
        //Log.w("beam","beam braked 1: " + getHowLongBeam1() + "Beam braked 2: " + getHowLongBeam2() + "Beam braked 3 " + getHowLongBeam3());

        light.setPosition(lightColor.value);
    }
    void calculateDistance() {
        double dx = targetX - shooterWorldX;
        double dy = targetY - shooterWorldY;
        shooterDistanceBackboard =  Math.sqrt(dx * dx + dy * dy);

         dx = targetX - currentX;
         dy = targetY -currentY;
        distanceToBackBoard =  Math.sqrt(dx * dx + dy * dy);
    }


    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

    public Pose getPose() {
        return pose;
    }
    public double getX() {
        return currentX;
    }


    public void calculateVelAndAcc() {
        double speedXAcel = 0;
        double speedYAcel = 0;
//        xAccRobot = speedXAcel * Math.cos(-currentHeading) - speedYAcel * Math.sin(-currentHeading);
//        yAccRobot = speedXAcel * Math.sin(-currentHeading) + speedYAcel * Math.cos(-currentHeading);
        xAccRobot = speedXAcel;
        yAccRobot = speedYAcel;

        velX = xVelocityFilter.getValue(robot.blob.getVelocityX());
        velY  = yVelocityFilter.getValue(robot.blob.getVelocityY());
//        xVelocityRobot = velX * Math.cos(-currentHeading) - velY * Math.sin(-currentHeading);
//        yVelocityRobot = velX * Math.sin(-currentHeading) + velY * Math.cos(-currentHeading);
        xVelocityRobot = velX;
        yVelocityRobot = velY;
    }
    public void toggleSOTM() {
        sotm = !sotm;
    }

    public double getMoveGoalX() {
        return virtualTargetX;
    }
    public double getMoveGoalY() {
        return virtualTargetY;
    }
    double distanceToBackBoard;
    double shooterDistanceBackboard;
    public double getDistanceToBackboard() {
        return distanceToBackBoard;
    }
    public double getShooterDistanceToBackboard() {
        return shooterDistanceBackboard;
    }
    double shooterAngle;

    public double getShooterAngleToTarget() {
        return shooterAngle;
    }

    public double getAngularVelocity() {
        return robot.blob.odo.getAngularVelocity();
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
    public void setLedColor(LightColor state) {
        lightColor = state;
    }
    /**
     * Returns the current estimated translational velocity of the robot in the
     * field frame (units per second).
     */
    public double getVelX() { return xVelocityRobot; }
    public double getVelY() { return yVelocityRobot; }

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

   public boolean isBreakBeamPos1Low(){
        return breakBeamPos1High;
    }
    public boolean isBreakBeamPos2Low(){
          return breakBeamPos2High;
     }
    public boolean isBreakBeamPos3Low(){
          return breakBeamPos3High;
     }

     public double getHowLongBeam3() {
        return System.currentTimeMillis() - firstTrueBeam3;
     }

    public double getHowLongBeam2() {
        return System.currentTimeMillis() - firstTrueBeam2;
    }

    public double getHowLongBeam1() {
        return System.currentTimeMillis() - firstTrueBeam1;
    }

    public double getDistanceFromPose(Pose pose) {
        double dx = targetX - pose.getX();
        double dy = targetY - pose.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }
    public double getShooterX() {
        return shooterWorldX;
    }
    public double getShooterY() {
        return shooterWorldY;
    }
    public void setUsePredictivePose(boolean use) {
        usePredictivePose = use;
    }

    public boolean areAllBeamsLowForTime(double msThreshold) {
        return getHowLongBeam1() > msThreshold + 20 && getHowLongBeam2() > msThreshold && getHowLongBeam3() > msThreshold;
    }

}
