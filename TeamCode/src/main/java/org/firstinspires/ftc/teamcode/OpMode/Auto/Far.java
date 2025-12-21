package org.firstinspires.ftc.teamcode.OpMode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Util.Globals.Alliance;
import org.firstinspires.ftc.teamcode.Util.Info;


@Config
@Autonomous(name = "Far")
public class Far extends LinearOpMode {


    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);

        if (Info.alliance == null) {
            Info.alliance = Alliance.BLUE;
        }

        robot.drive.setStartingPose(new Pose());

        waitForStart();

        sleep(FarConstants.waitMillis);

        double adjustedY = FarConstants.targetY * (Info.alliance == Alliance.BLUE ? 1 : -1);
        Pose start = new Pose(FarConstants.startX, FarConstants.startY * (Info.alliance == Alliance.BLUE ? 1 : -1),FarConstants.headingStartBlue * (Info.alliance == Alliance.BLUE ? 1 : -1) );

        Pose target = new Pose(FarConstants.targetX, adjustedY, FarConstants.targetHeading * (Info.alliance == Alliance.BLUE ? 1 : -1));

        PathChain path = robot.drive.pathBuilder()
                .addPath(new BezierLine(start, target))
                .setLinearHeadingInterpolation(start.getHeading(), target.getHeading())
                .build();

        robot.drive.followPath(path);

        while (opModeIsActive() && !robot.drive.isBusy()) {
            robot.update();
        }

        while (opModeIsActive()) {
            robot.update();
        }
    }
}
