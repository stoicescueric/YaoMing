package org.firstinspires.ftc.teamcode.Hardware.Drive.p2p;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Drive.p2p.control.PDFSConstants;
import org.firstinspires.ftc.teamcode.Hardware.Drive.p2p.control.PDFSController;
import org.firstinspires.ftc.teamcode.Hardware.Drive.p2p.control.Pose;
import org.firstinspires.ftc.teamcode.Hardware.Drive.p2p.control.SimpleMath;
import org.firstinspires.ftc.teamcode.Hardware.Drive.p2p.follower.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Drive.p2p.follower.Follower;
import org.firstinspires.ftc.teamcode.Hardware.Drive.p2p.follower.PidToPointFollower;
import org.firstinspires.ftc.teamcode.Hardware.Drive.p2p.localizer.PinpointLocalizer;


@Config
public class MecanumDrive extends Drivetrain {

    public static boolean headingLock = false;

    public static boolean leftFrontReversed = true;
    public static boolean rightFrontReversed = false;
    public static boolean leftRearReversed = true;
    public static boolean rightRearReversed = false;
    public static boolean coastInTeleop = false;

    public static double K_STATIC = 0;

    public static double translationalMaxPower = 1;
    public static double rotationalMaxPower = 1;

    private boolean headingManuallyControlled = false;
    private final ElapsedTime headingTimer = new ElapsedTime();
    private double headingVelocity = 0;
    private Pose lastPose = new Pose();
    private double targetHeading = 0;

    public PDFSController headingController;

    private final DcMotorEx leftFront;
    private final DcMotorEx rightFront;
    private final DcMotorEx leftRear;
    private final DcMotorEx rightRear;

    public MecanumDrive(HardwareMap hw) {
        driveMode = DriveMode.ROBOT_CENTRIC;

        forwardConstants = new PDFSConstants(0, 0, 0, 0);
        strafeConstants = new PDFSConstants(0, 0, 0, 0);
        headingConstants = new PDFSConstants(0, 0, 0, 0);

        localizer = new PinpointLocalizer(hw);
        follower = new PidToPointFollower(this);
        followerTolerance = 1.75; //distance in cm before atTarget() returns true

        leftFront = hw.get(DcMotorEx.class, "fl");
        rightFront = hw.get(DcMotorEx.class, "fr");
        leftRear =hw.get(DcMotorEx.class, "bl");
        rightRear =hw.get(DcMotorEx.class, "br");

        leftFront.setDirection(leftFrontReversed ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(rightFrontReversed ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);
        leftRear.setDirection(leftRearReversed ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);
        rightRear.setDirection(rightRearReversed ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);

        headingController = new PDFSController(headingConstants);
    }

    public void setDriveMode(DriveMode driveMode) {
        this.driveMode = driveMode;
        if (driveMode != DriveMode.AUTONOMOUS && coastInTeleop) {
            leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            leftRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            rightRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        } else {
            leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            leftRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rightRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }
    }

    public Pose getPosition() {
        return localizer.getPose();
    }

    public void setPosition(Pose pose) {
        localizer.setPose(pose);
    }

    public Pose getVelocity() {
        return localizer.getVelocity();
    }

    public void drive(double x, double y, double heading) {
        x = SimpleMath.clamp(x, -1, 1);
        y = SimpleMath.clamp(y, -1, 1);
        heading = SimpleMath.clamp(heading, -1, 1);

        x *= translationalMaxPower;
        y *= translationalMaxPower;
        heading *= rotationalMaxPower;

        if (driveMode == DriveMode.FIELD_CENTRIC) {
            double rotated_x = x * Math.cos(localizer.getPose().getHeading())
                    - y * Math.sin(localizer.getPose().getHeading());
            double rotated_y = x * Math.sin(localizer.getPose().getHeading())
                    + y * Math.cos(localizer.getPose().getHeading());
            x = rotated_x;
            y = rotated_y;
        }

        if (heading != 0) {
            headingManuallyControlled = true;
            //heading += K_STATIC*Math.signum(heading); //compensate for static friction for more precise control?
        } else if (headingLock) {
            if (/* headingVelocity < Math.toRadians(10) && */ headingManuallyControlled) {
                headingManuallyControlled = false;
                targetHeading = localizer.getPose().getHeading();
            }
            double delta = Math.atan2(Math.sin(targetHeading - localizer.getPose().getHeading()), Math.cos(targetHeading - localizer.getPose().getHeading()));
            heading = headingController.calculate(0, -delta);
        }

        double denominator = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(heading), 1);

        leftFront.setPower((x - y - heading) / denominator);
        rightFront.setPower((x + y + heading) / denominator);
        leftRear.setPower((x + y - heading) / denominator);
        rightRear.setPower((x - y + heading) / denominator);
    }

    public Follower getFollower() {
        return follower;
    }

    public void setMotorPowers(
            double leftFront,
            double rightFront,
            double leftRear,
            double rightRear) {
        this.leftFront.setPower(leftFront);
        this.rightFront.setPower(rightFront);
        this.leftRear.setPower(leftRear);
        this.rightRear.setPower(rightRear);
    }

    public void update() {
        headingController.setConstants(headingConstants);


        headingVelocity = (lastPose.getHeading() - localizer.getPose().getHeading()) / headingTimer.seconds();
        headingTimer.reset();

        localizer.update();
        lastPose = localizer.getPose();

        if (driveMode == DriveMode.AUTONOMOUS) {
            double voltage = 0;
            Pose followVector = follower.getFollowVector().scale(12 / voltage);
            drive(followVector.getX(), followVector.getY(), followVector.getHeading());
        }
    }
}