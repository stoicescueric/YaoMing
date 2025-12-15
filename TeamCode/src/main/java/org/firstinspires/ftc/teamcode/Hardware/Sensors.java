package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.geometry.Pose;

public class Sensors {
    private Robot robot;

    Pose pose;
    double currentX,currentY,currentHeading;

    private double currentVelocityShooter = 0;
    private double voltage;
    private LynxModule controlHub, expansionHub;

    public double readVoltageTime = 0;


    public Sensors(Robot robot) {
        this.robot = robot;
        initSensors();
    }
    public static double targetX = -64.1;
    public static double targetY = -60.1;
    private void initSensors() {
        controlHub = robot.hw.get(LynxModule.class, "Control Hub");
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        expansionHub = robot.hw.get(LynxModule.class, "Expansion Hub 2");
        expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        voltage = robot.hw.voltageSensor.iterator().next().getVoltage();
        readVoltageTime = System.currentTimeMillis();
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


        currentVelocityShooter = robot.outtake.getShooterMotor().getVelocity();

        if(System.currentTimeMillis() - readVoltageTime > 250) {
            voltage = robot.hw.voltageSensor.iterator().next().getVoltage();
            readVoltageTime = System.currentTimeMillis();
        }
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

    public double getY() {
        return currentY;
    }
    public double getHeading() {
        return currentHeading;
    }
    public double getVelocity() {
        return currentVelocityShooter;
    }

}
