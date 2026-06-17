package org.firstinspires.ftc.teamcode.blob.localization;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.blob.constants.BlobConstants;
import org.firstinspires.ftc.teamcode.blob.math.LowPassFilter;

import java.util.ArrayDeque;

@Config
public class Odometry {

    private final GoBildaPinpointDriver odo;

    // --- State Variables ---
    public double x, y, heading, realHead;
    public double xVelocity, yVelocity, headingVelocity;
    public double speedTranslational;
    public double predictedX, predictedY;

    // --- Robot-Centric Velocities & Glide ---
    public double xRobotVelocity, yRobotVelocity;
    public double forwardGlide, lateralGlide, xGlide, yGlide;

    // --- Acceleration State ---
    public double xAcc, yAcc;
    private double lastVelocityX, lastVelocityY;
    private final ElapsedTime loopTimer = new ElapsedTime();

    // --- Constants & Config ---
    public double zpam = BlobConstants.zpam;
    public double xDeceleration = BlobConstants.xDeceleration;
    public double yDeceleration = BlobConstants.yDeceleration;

    // Time constants (seconds) for the dt-aware low-pass filters. Larger tau = more smoothing.
    // These supersede the old fixed alphas: alpha = dt / (tau + dt) is recomputed every loop,
    // so the cutoff stays constant regardless of loop frequency.
    public static double tauTranslational = 0.0017;     // ~ old alpha 0.9 @ 15ms loop
    public static double tauTranslationalAcc = 0.0225;  // ~ old alpha 0.4 @ 15ms loop
    public static double tauAngular = 0.0038;           // ~ old alpha 0.8 @ 15ms loop

    // --- Filters ---
    private final LowPassFilter xVelocityFilter = LowPassFilter.fromTimeConstant(tauTranslational, 0);
    private final LowPassFilter yVelocityFilter = LowPassFilter.fromTimeConstant(tauTranslational, 0);
    private final LowPassFilter hVelocityFilter = LowPassFilter.fromTimeConstant(tauAngular, 0);

    private final LowPassFilter xAccFilter = LowPassFilter.fromTimeConstant(tauTranslationalAcc, 0);
    private final LowPassFilter yAccFilter = LowPassFilter.fromTimeConstant(tauTranslationalAcc, 0);


    public Odometry(HardwareMap hardwareMap) {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, BlobConstants.pinpointName);
        odo.setEncoderDirections(BlobConstants.xPodDirection, BlobConstants.yPodDirection);
        odo.setEncoderResolution(BlobConstants.podType);
        odo.setOffsets(BlobConstants.xOffset, BlobConstants.yOffset, DistanceUnit.INCH);

        odo.resetPosAndIMU();

        // Note: Blocking the thread here is okay during init, but ensure it doesn't cause a watchdog timeout.
        try {
            sleep(650);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt(); // Restore interrupted status
        }

        loopTimer.reset();
    }

    public void update() {
        odo.update();

        // 1. Update Position
        x = odo.getPosX(DistanceUnit.INCH);
        y = odo.getPosY(DistanceUnit.INCH);
        heading = odo.getHeading(AngleUnit.RADIANS);

        // Map heading to 0 -> 2PI
        realHead = (heading < 0) ? Math.abs(heading) : (2 * Math.PI - heading);

        // 2. Safely Update Kinematics if valid data exists
        if (!Double.isNaN(x) && !Double.isNaN(y)) {
            calculateVelAndAcc();
            updateGlide();

            predictedX = x + xGlide;
            predictedY = y + yGlide;
        }
    }

    private void calculateVelAndAcc() {
        // One loop dt drives every filter so the cutoff is loop-rate independent.
        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (dt < 1e-4) dt = 1e-4; // guard against zero/negative dt

        // Push live dashboard tuning into the filters.
        xVelocityFilter.setTimeConstant(tauTranslational);
        yVelocityFilter.setTimeConstant(tauTranslational);
        hVelocityFilter.setTimeConstant(tauAngular);
        xAccFilter.setTimeConstant(tauTranslationalAcc);
        yAccFilter.setTimeConstant(tauTranslationalAcc);

        // Read & Filter Velocities (dt-aware EMA: alpha = dt / (tau + dt))
        xVelocity = xVelocityFilter.getValue(odo.getVelX(DistanceUnit.INCH), dt);
        yVelocity = yVelocityFilter.getValue(odo.getVelY(DistanceUnit.INCH), dt);
        headingVelocity = hVelocityFilter.getValue(odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS), dt);
        speedTranslational = Math.hypot(xVelocity, yVelocity);

        // Acceleration from filtered velocity, then low-passed with the same dt.
        double rawXAcc = (xVelocity - lastVelocityX) / dt;
        double rawYAcc = (yVelocity - lastVelocityY) / dt;

        xAcc = xAccFilter.getValue(rawXAcc, dt);
        yAcc = yAccFilter.getValue(rawYAcc, dt);

        // Save states for next loop
        lastVelocityX = xVelocity;
        lastVelocityY = yVelocity;
    }

    public double xVelRobotCentric,yVelRobotCentricl;
    private void updateGlide() {
        zpam = BlobConstants.zpam;

        // Convert Field-Centric velocity to Robot-Centric velocity
        double cosH = Math.cos(-heading);
        double sinH = Math.sin(-heading);
        xVelRobotCentric = xVelocity * cosH - yVelocity * sinH;
        yVelRobotCentricl = xVelocity * sinH + yVelocity * cosH;

        // Kinematic stopping distance: d = v^2 / (2a)
        forwardGlide = Math.signum(xVelRobotCentric) * (xVelRobotCentric * xVelRobotCentric) / (2.0 * xDeceleration);
        lateralGlide = Math.signum(yVelRobotCentricl) * (yVelRobotCentricl * yVelRobotCentricl) / (2.0 * yDeceleration);

        // Convert Robot-Centric glide back to Field-Centric glide
        cosH = Math.cos(heading);
        sinH = Math.sin(heading);
        xGlide = forwardGlide * cosH - lateralGlide * sinH;
        yGlide = forwardGlide * sinH + lateralGlide * cosH;
    }


    public void calibrate() { odo.recalibrateIMU(); }
    public void reset() { odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0)); }
    public void setPose(Pose pose) { odo.setPosition(new Pose2D(DistanceUnit.INCH, pose.getX(), pose.getY(), AngleUnit.RADIANS, pose.getHeading())); }

    public Pose getPose() { return new Pose(x, y, heading); }
    public Vector2d getAccVector() { return new Vector2d(xAcc, yAcc); }
    public double getAccX() { return xAcc; }
    public double getAccY() { return yAcc; }

    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeading() { return heading; }
    public double getRealHeading() { return realHead; }

    public double getVelX() { return xVelocity; }
    public double getVelY() { return yVelocity; }
    public double getAngularVelocity() { return headingVelocity; }
    public double getSpeedTranslational() { return speedTranslational; }
}