package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.geometry.Pose;
@Config
public class Sensors {
    private Robot robot;

    Pose pose;
    double currentX,currentY,currentHeading;

    private double currentVelocityShooter = 0;
    private double voltage;
    private LynxModule controlHub, expansionHub;

    public long readVoltageTime = 0;
    private long lastUpdateTimeMs = 0;
    private double cycleRateHz = 0.0;
    private static final double CYCLE_SMOOTHING_ALPHA = 0.2;


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
        lastUpdateTimeMs = readVoltageTime;
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
        long now = System.currentTimeMillis();
        if (lastUpdateTimeMs != 0) {
            long dtMs = now - lastUpdateTimeMs;
            if (dtMs > 0) {
                double instHz = 1000.0 / dtMs;
                if (cycleRateHz == 0.0) {
                    cycleRateHz = instHz;
                } else {
                    cycleRateHz = CYCLE_SMOOTHING_ALPHA * instHz + (1.0 - CYCLE_SMOOTHING_ALPHA) * cycleRateHz;
                }
            }
        }
        lastUpdateTimeMs = now;
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

    public double getCycleRateHz() {
        return cycleRateHz;
    }

}
