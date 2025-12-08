package org.firstinspires.ftc.teamcode.Hardware.Drive.p2p.path;


import org.firstinspires.ftc.teamcode.Hardware.Drive.p2p.control.Pose;

public interface Path {
    Pose getClosestPose(Pose pose);
    Pose getDistanceFromEnd(Pose pose);
    Pose getEndPose();
}