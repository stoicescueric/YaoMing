package org.firstinspires.ftc.teamcode.Hardware.Drive.p2p.follower;


import org.firstinspires.ftc.teamcode.Hardware.Drive.p2p.control.Pose;

public interface Follower {
    Follower add(Pose... points);
    Follower clear();
    boolean atTarget();
    Pose getFollowVector();
}