package org.firstinspires.ftc.teamcode.Hardware.Drive.p2p.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware.Drive.p2p.MecanumDrive;
import org.firstinspires.ftc.teamcode.Hardware.Drive.p2p.control.Pose;
import org.firstinspires.ftc.teamcode.Hardware.Drive.p2p.follower.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Drive.p2p.follower.Follower;


@Config
@Autonomous(group = "Tuners")
public class P2PTest extends OpMode {

    public static double x,y,heading;
    Drivetrain drivetrain;
    Follower follower;


    @Override
    public void init() {
        drivetrain = new MecanumDrive(hardwareMap);
        follower = drivetrain.getFollower();
        drivetrain.localizer.setPose(new Pose());
        drivetrain.setDriveMode(Drivetrain.DriveMode.AUTONOMOUS);
    }

    @Override
    public void loop() {

    }
}