package org.firstinspires.ftc.teamcode.blob.driveTrain;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.Util.Caching.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.blob.constants.BlobConstants;
import org.firstinspires.ftc.teamcode.blob.localization.Odometry;
import org.firstinspires.ftc.teamcode.blob.math.PIDControllerBlob;


@Config
public class Blob {


    DcMotorEx leftFront, leftBack, rightFront, rightBack;

    public double targetX, targetY, x = 0, y = 0;
    public double targetHeading, rotation, realHeading, targetHeadingBlob;
    public double lateralMultiplier = BlobConstants.lateralMultiplier;
    public double error;
    public Odometry odo;
    public boolean isStuckFailsafe = false;

    double prevX, prevY, prevH;

    double headingThreshold;
    double totalDistance, travelDistance;
    public double progress;
    boolean useHeadingThreshold = false;


    double frontLeftPower;
    double backLeftPower;
    double frontRightPower;
    double backRightPower;
    public boolean hasPower = true;
    public boolean isVelocityNull = false;
    public double maxPower = 1;


    public double kP = BlobConstants.kP, kI = BlobConstants.kI, kD = BlobConstants.kD;
    public double hP = BlobConstants.hP, hI = BlobConstants.hI, hD = BlobConstants.hD;

    public PIDControllerBlob controllerX = new PIDControllerBlob(kP, kI, kD);
    public PIDControllerBlob controllerY = new PIDControllerBlob(kP, kI, kD);
    public PIDControllerBlob controllerHeading = new PIDControllerBlob(hP, hI, hD);
    public enum State{
        DRIVE , PID;
    }
    State state;

    public Blob(HardwareMap hardwareMap,State initialState){

        state=initialState;
        odo = new Odometry(hardwareMap);
        leftFront = hardwareMap.get(DcMotorEx.class, BlobConstants.leftFrontName);
        leftBack = hardwareMap.get(DcMotorEx.class, BlobConstants.leftBackName);
        rightFront = hardwareMap.get(DcMotorEx.class, BlobConstants.rightFrontName);
        rightBack = hardwareMap.get(DcMotorEx.class, BlobConstants.rightBackName);



        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //setTargetVector(0, 0, 0);
    }

    public double returnFrVelocity() {
        return rightFront.getVelocity();
    }


    public boolean inPosition(){

        double heading = odo.getHeading();
        if(heading < 0) realHeading = Math.abs(heading);
        else realHeading = 2 * Math.PI - heading;

        error = targetHeading - realHeading;
        if(Math.abs(error) > Math.PI) {
            error = -Math.signum(error) * (2 * Math.PI - Math.abs(error));
        }
        if(Math.abs(targetX - odo.getX()) < BlobConstants.xDefTresh &&
                Math.abs(targetY - odo.getY()) < BlobConstants.yDefTresh &&
                Math.abs(error) < BlobConstants.hDefTresh){
            //Log.w("blob error","x " + (targetX - odo.getX()));
            //Log.w("blob error","y " + (targetY - odo.getY()));
            //Log.w("blob error","heading " + error);
            return true;
        }
        return false;
    }

    public void turnToDegrees(double degrees){
        targetHeading = Math.toRadians(degrees);
    }

    public void turnToRadians(double radians){
        targetHeading = radians;
    }

    public boolean inPosition(double x, double y, double hError){

        double heading = odo.getHeading();
        if(heading < 0) realHeading = Math.abs(heading);
        else realHeading = 2 * Math.PI - heading;

        error = targetHeading - realHeading;
        if(Math.abs(error) > Math.PI) {
            error = -Math.signum(error) * (2 * Math.PI - Math.abs(error));
        }

        return Math.abs(targetX - odo.getX()) < x &&
                Math.abs(targetY - odo.getY()) < y &&
                Math.abs(error) < hError;

    }

    public boolean inPosition(double x, double y){

        double heading = odo.getHeading();
        if(heading < 0) realHeading = Math.abs(heading);
        else realHeading = 2 * Math.PI - heading;

        error = targetHeading - realHeading;
        if(Math.abs(error) > Math.PI) {
            error = -Math.signum(error) * (2 * Math.PI - Math.abs(error));
        }

        return Math.abs(targetX - odo.getX()) < x &&
                Math.abs(targetY - odo.getY()) < y &&
                Math.abs(error) < 0.1;

    }

    public void setTargetVector(double x , double y , double rx){

        //Log.w("Blob Values","" + x +" " + y + " " + rx );
        x *= lateralMultiplier;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx) , 1);
        frontLeftPower = ((y + x + rx) / denominator) * maxPower;
        backLeftPower = ((y - x + rx) / denominator) * maxPower;
        frontRightPower = ((y - x - rx) / denominator) * maxPower;
        backRightPower = ((y + x - rx) / denominator) * maxPower;

        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower );
        rightFront.setPower(frontRightPower );
        rightBack.setPower(backRightPower ) ;

    }

    public void setTargetPosition(double targetX , double targetY , double targetHeading){

        this.targetX = targetX;
        this.targetY = targetY;
        this.targetHeading =targetHeading-Math.floor((targetHeading/ (Math.PI*2)))*Math.PI*2;
        useHeadingThreshold = false;

    }

    public void setTargetPosition(double targetX , double targetY){

        this.targetX = targetX;
        this.targetY = targetY;
        useHeadingThreshold = false;

    }

    public void setTargetPosition(Pose pose){

        targetX = pose.getX();
        targetY = pose.getY();
        targetHeading = pose.getHeading();
        useHeadingThreshold = false;

    }

    public boolean isStuck(){
        hasPower = frontRightPower != 0 || frontLeftPower != 0 || backRightPower != 0 || backLeftPower != 0;
        isVelocityNull = odo.getVelX() < 0.1 && odo.getVelY() < 0.1;
        return hasPower && isVelocityNull;
    }

    public void setTargetPosition(Pose pose, double headingThreshold, double prevH){

        targetX = pose.getX();
        targetY = pose.getY();
        targetHeadingBlob = pose.getHeading();
        this.headingThreshold = headingThreshold;
        useHeadingThreshold = true;
        prevX = odo.getX();
        prevY = odo.getY();
        this.prevH = prevH;

    }
    public void setMode(State state)
    {
        this.state=state;
    }

    public double getVelocityX() {
        return odo.xVelocity;
    }
    public double getVelocityY() {
        return odo.yVelocity;
    }

    public void update()
    {
        odo.update();
        if (state !=State.PID) {return;}


        controllerX.kp = BlobConstants.kP;
        controllerY.kp = BlobConstants.kP;

        controllerX.ki = 0;
        controllerY.ki = 0;

        controllerX.kd = BlobConstants.kD;
        controllerY.kd = BlobConstants.kD;;

        controllerHeading.kp = BlobConstants.hP;
        controllerHeading.ki = 0;
        controllerHeading.kd = BlobConstants.hD;

        if(Double.isNaN(odo.x) || Double.isNaN(odo.y) || Double.isNaN(odo.heading)){
            return;
        }

        x = controllerX.calculate(targetX, odo.x);
        y = -controllerY.calculate(targetY , odo.y);

        double heading = odo.getHeading();
        if(heading < 0) realHeading = Math.abs(heading);
        else realHeading = 2 * Math.PI - heading;

        error = targetHeading - realHeading;
        if(Math.abs(error) > Math.PI) error = -Math.signum(error) * (2 * Math.PI - Math.abs(error));
        rotation = controllerHeading.calculate(error , 0);

        totalDistance = Math.sqrt(Math.pow(targetX - prevX, 2) + Math.pow(targetY - prevY, 2));
        travelDistance = Math.sqrt(Math.pow(odo.getX() - prevX, 2) + Math.pow(odo.getY() - prevY, 2));
        progress = Math.min(Math.max(travelDistance / totalDistance, 0), 1);

        setTargetVector(y * Math.cos(-heading) - x * Math.sin(-heading), y * Math.sin(-heading) + x * Math.cos(-heading) , rotation);
    }

}