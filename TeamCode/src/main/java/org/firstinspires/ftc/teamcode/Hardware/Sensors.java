package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Sensors {
    private Robot robot;

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
        currentVelocityShooter = robot.outtake.getShooterMotor().getVelocity();


        if(readVoltageTime - System.currentTimeMillis() > 500) {
            voltage = robot.hw.voltageSensor.iterator().next().getVoltage();
            readVoltageTime = System.currentTimeMillis();
        }
    }

    public double getVelocity() {
        return currentVelocityShooter;
    }

}
