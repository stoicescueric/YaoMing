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
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.blob.constants.BlobConstants;
import org.firstinspires.ftc.teamcode.blob.math.LowPassFilter;


@Config
public class Odometry {

    GoBildaPinpointDriver odo;
    VoltageSensor vs;

    public double x, y, heading, xVelocity, yVelocity, predictedX, predictedY;
    public double speedTranslational;
    public double speedOverAll;

    Vector2d robotVelocityVector;
    public double headingVelocity;
    double voltage;
    int i = 10;
    public Odometry(HardwareMap hardwareMap){
        odo = hardwareMap.get(GoBildaPinpointDriver.class, BlobConstants.pinpointName);
        odo.setEncoderDirections(BlobConstants.xPodDirection, BlobConstants.yPodDirection);
        odo.setEncoderResolution(BlobConstants.podType);


        odo.setOffsets(BlobConstants.xOffset, BlobConstants.yOffset, DistanceUnit.INCH);
        odo.resetPosAndIMU();
        try{
            sleep(650);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public void calibrate(){
        odo.recalibrateIMU();
    }

    public double getHeading(){
        return heading;
    }
    public double getRealHeading(){
        return realHead;
    }

    public double getX(){
        return x;
    }

    public double getY(){
        return y;
    }

    public void reset(){
        odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0));
    }
    public void setPose(Pose pose){
        odo.setPosition(new Pose2D(DistanceUnit.INCH, pose.getX(), pose.getY(), AngleUnit.RADIANS, pose.getHeading()));
    }
    public double getAngularVelocity(){

        return headingVelocity;
    }

    public double getVelX(){
        return xRobotVelocity;
    }

    public double getVelY(){
        return yRobotVelocity;
    }
    public Pose getPose() {
        return new Pose( x, y,heading);
    }

    public static double filterParameterTranslational = 0.9;
    public static double filterParameterTranslationalAcc = 0.6;
    public static double filterParameterAngular = 0.8;
    private  final LowPassFilter xVelocityFilter = new LowPassFilter(filterParameterTranslational, 0);
    private final LowPassFilter yVelocityFilter = new LowPassFilter(filterParameterTranslational, 0);
    private  final LowPassFilter xAccFilter = new LowPassFilter(filterParameterTranslationalAcc, 0);
    private final LowPassFilter yAccFilter = new LowPassFilter(filterParameterTranslationalAcc, 0);
    private final LowPassFilter hVelocityFilter = new LowPassFilter(filterParameterAngular, 0);

    public double zpam = BlobConstants.zpam;
    public double xDeceleration = BlobConstants.xDeceleration, yDeceleration = BlobConstants.yDeceleration;
    public double xRobotVelocity, yRobotVelocity;
    public double forwardGlide, lateralGlide;
    public double xGlide, yGlide;
    public double realHead;

    public Vector2d getAccVector() {
        return new Vector2d(xAcc, yAcc);
    }

    ElapsedTime timerAcc = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    public double xAcc, yAcc;
    public double lastVelocityX, lastVelocityY;
    public void calculateVelAndAcc() {
        xVelocity = xVelocityFilter.getValue(odo.getVelX(DistanceUnit.INCH));
        yVelocity = yVelocityFilter.getValue(odo.getVelY(DistanceUnit.INCH));

        xAcc = xAccFilter.getValue((xVelocity - lastVelocityX) / timerAcc.seconds());
        yAcc = yAccFilter.getValue((yVelocity - lastVelocityY) / timerAcc.seconds());
        timerAcc.reset();
        robotVelocityVector = new Vector2d(xVelocity, yVelocity);
        speedTranslational = robotVelocityVector.magnitude();
        headingVelocity = hVelocityFilter.getValue(odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
        lastVelocityX = xVelocity;
        lastVelocityY = yVelocity;
    }
    private void updateGlide(){
        zpam = BlobConstants.zpam;
        xRobotVelocity = xVelocity * Math.cos(-heading) - yVelocity * Math.sin(-heading);
        yRobotVelocity = xVelocity * Math.sin(-heading) + yVelocity * Math.cos(-heading);

        forwardGlide = Math.signum(xRobotVelocity) * xRobotVelocity * xRobotVelocity / (2.0 * xDeceleration * zpam);
        lateralGlide = Math.signum(yRobotVelocity) * yRobotVelocity * yRobotVelocity / (2.0 * yDeceleration * zpam);

        xGlide = forwardGlide * Math.cos(heading) - lateralGlide * Math.sin(heading);
        yGlide = forwardGlide * Math.sin(heading) + lateralGlide * Math.cos(heading);
    }

    public double getSpeedTranslational() {
        return speedTranslational;
    }
    public  void update()
    {

        odo.update();

        heading = odo.getHeading(AngleUnit.RADIANS);

        if(heading < 0) realHead = Math.abs(heading);
        else realHead = 2 * Math.PI - heading;

        x = odo.getPosX(DistanceUnit.INCH);

        y = odo.getPosY(DistanceUnit.INCH);

        if(!Double.isNaN(x) && !Double.isNaN(y)) {
            calculateVelAndAcc();
            updateGlide();
            predictedX = x + xGlide;
            predictedY = y + yGlide;
        }

    }

}