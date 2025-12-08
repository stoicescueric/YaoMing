package org.firstinspires.ftc.teamcode.Hardware.Drive.p2p.localizer;


import org.firstinspires.ftc.teamcode.Hardware.Drive.p2p.control.Pose;

public interface Localizer {
    Pose getPose();
    Pose getVelocity();
    void setPose(Pose pose);
    void update();
}