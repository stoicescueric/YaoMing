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

    private void initSensors() {
        controlHub = robot.hw.get(LynxModule.class, "Control Hub");
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        expansionHub = robot.hw.get(LynxModule.class, "Expansion Hub 2");
        expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        voltage = robot.hw.voltageSensor.iterator().next().getVoltage();
        readVoltageTime = System.currentTimeMillis();
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
