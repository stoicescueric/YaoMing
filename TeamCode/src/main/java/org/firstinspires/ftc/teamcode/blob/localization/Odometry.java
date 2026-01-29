package org.firstinspires.ftc.teamcode.blob.localization;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.blob.constants.BlobConstants;
import org.firstinspires.ftc.teamcode.blob.math.LowPassFilter;


@Config
public class Odometry {

    GoBildaPinpointDriver odo;
    VoltageSensor vs;

    public double x, y, heading, xVelocity, yVelocity, predictedX, predictedY;
    double voltage;
    int i = 10;

    public Odometry(HardwareMap hardwareMap){
        vs = hardwareMap.voltageSensor.iterator().next();
        odo = hardwareMap.get(GoBildaPinpointDriver.class, BlobConstants.pinpointName);
        odo.setEncoderDirections(BlobConstants.xPodDirection, BlobConstants.yPodDirection);
        odo.setEncoderResolution(BlobConstants.podType);
        odo.setOffsets(BlobConstants.xOffset, BlobConstants.yOffset, DistanceUnit.MM);
        odo.resetPosAndIMU();
    }

    public void calibrate(){
        odo.recalibrateIMU();
    }

    public double getHeading(){
        return heading;
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

    public double getVelX(){
        return odo.getVelX(DistanceUnit.INCH);
    }

    public double getVelY(){
        return odo.getVelY(DistanceUnit.INCH);
    }
    public Pose getPose() {
        return new Pose( x, y,heading);
    }

    public static double filterParameter = 0.8;
    private static final LowPassFilter xVelocityFilter = new LowPassFilter(filterParameter, 0);
    private final LowPassFilter yVelocityFilter = new LowPassFilter(filterParameter, 0);

    public double zpam = BlobConstants.zpam;
    public double xDeceleration = BlobConstants.xDeceleration, yDeceleration = BlobConstants.yDeceleration;
    public double xRobotVelocity, yRobotVelocity;
    public double forwardGlide, lateralGlide;
    public double xGlide, yGlide;

    private void updateGlide(){

        xRobotVelocity = xVelocity * Math.cos(-heading) - yVelocity * Math.sin(-heading);
        yRobotVelocity = xVelocity * Math.sin(-heading) + yVelocity * Math.cos(-heading);

        forwardGlide = Math.signum(xRobotVelocity) * xRobotVelocity * xRobotVelocity / (2.0 * xDeceleration * zpam);
        lateralGlide = Math.signum(yRobotVelocity) * yRobotVelocity * yRobotVelocity / (2.0 * yDeceleration * zpam);

        xGlide = forwardGlide * Math.cos(heading) - lateralGlide * Math.sin(heading);
        yGlide = forwardGlide * Math.sin(heading) + lateralGlide * Math.cos(heading);
    }

    public  void update()
    {

        if(i == 10){
            voltage = vs.getVoltage();
            i = 0;
        }

        odo.update();

        heading = odo.getHeading(AngleUnit.RADIANS);

        x = odo.getPosX(DistanceUnit.INCH);

        y = odo.getPosY(DistanceUnit.INCH);

        if(!Double.isNaN(x) && !Double.isNaN(y)) {
            xVelocity = xVelocityFilter.getValue(odo.getVelX(DistanceUnit.INCH));
            yVelocity = yVelocityFilter.getValue(odo.getVelY(DistanceUnit.INCH));
            updateGlide();
            predictedX = x + xGlide;
            predictedY = y + yGlide;
        }

        i++;
    }

}
