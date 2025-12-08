package org.firstinspires.ftc.teamcode.Hardware.Drive.p2p.follower;


import org.firstinspires.ftc.teamcode.Hardware.Drive.p2p.control.PDFSConstants;
import org.firstinspires.ftc.teamcode.Hardware.Drive.p2p.localizer.Localizer;

public abstract class Drivetrain {
    public enum DriveMode {
        FIELD_CENTRIC,
        ROBOT_CENTRIC,
        AUTONOMOUS
    }
    public static double followerTolerance;
    public static PDFSConstants forwardConstants;
    public static PDFSConstants strafeConstants;
    public static PDFSConstants headingConstants;
    public Localizer localizer;
    public Follower follower;
    public DriveMode driveMode;
    abstract public void setMotorPowers(double leftFront, double rightFront, double leftRear, double rightRear);
    abstract public void setDriveMode(DriveMode driveMode);
    abstract public Follower getFollower();
    abstract public void drive(double forwards, double strafe, double turn);
    abstract public void update();
}