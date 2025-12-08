package org.firstinspires.ftc.teamcode.Hardware.Drive.p2p.localizer;


import static com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.FORWARD;
import static com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.REVERSED;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.Drive.p2p.control.Pose;
import org.firstinspires.ftc.teamcode.Util.Filters.LowPassFilter;


@Config
public class  PinpointLocalizer implements Localizer {
    private final static int bus = 0;

    public static double xOffset = -84;
    public static double yOffset = -168;
    public  static boolean forwardEncoderReversed = false;
    public static  boolean strafeEncoderReversed = false;
    public static double filterParameter = 0.8;
    private static final LowPassFilter xVelocityFilter = new LowPassFilter(filterParameter);
    private static final LowPassFilter yVelocityFilter = new LowPassFilter(filterParameter);

    Pose currentPose;
    Pose currentVelocity;
    double x,y,heading;
    double predictedX,predictedY;
    double xVelcocity,yVelocity,headingVelocity;

    public static double xDeceleration = 120 * 25.4, yDeceleration = 150 * 25.4;
    public static double xRobotVelocity, yRobotVelocity;
    public static double forwardGlide, lateralGlide;
    public static double xGlide, yGlide;

    private final GoBildaPinpointDriver pinpoint;

    public PinpointLocalizer(HardwareMap hw) {
        if (bus < 0 || bus > 3) throw new IllegalArgumentException("Port must be between 0 and 3");
        pinpoint =  hw.get(GoBildaPinpointDriver.class,"pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(xOffset, yOffset, DistanceUnit.MM);
        pinpoint.setEncoderDirections(
                (forwardEncoderReversed ? REVERSED : FORWARD),
                (strafeEncoderReversed ? REVERSED : FORWARD)
        );
        pinpoint.recalibrateIMU();
        pinpoint.resetPosAndIMU();
    }

    @Override
    public Pose getPose() {
        return (Pose)pinpoint.getPosition();
    }



    public Pose predictedPose() {
        return new Pose(predictedX, predictedY, heading);
    }
    @Override
    public Pose getVelocity() {
        return new Pose(pinpoint.getVelX(DistanceUnit.MM), pinpoint.getVelY(DistanceUnit.MM), pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
    }



    @Override
    public void setPose(Pose pose) {
        pinpoint.setPosition(pose);
    }

    public void updateGlide() {
        xRobotVelocity = xVelcocity * Math.cos(-heading) - yVelocity * Math.sin(-heading);
        yRobotVelocity = xVelcocity * Math.sin(-heading) + yVelocity * Math.cos(-heading);

        forwardGlide = Math.signum(xRobotVelocity) * xRobotVelocity * xRobotVelocity / (2.0 * xDeceleration);
        lateralGlide = Math.signum(yRobotVelocity) * yRobotVelocity * yRobotVelocity / (2.0 * yDeceleration);

        xGlide = forwardGlide * Math.cos(heading) - lateralGlide * Math.sin(heading);
        yGlide = forwardGlide * Math.sin(heading) + lateralGlide * Math.cos(heading);
    }
    @Override
    public void update() {
        pinpoint.update();

        currentPose = (Pose)pinpoint.getPosition();
        currentVelocity = getVelocity();

        x = currentPose.getX();
        y = currentPose.getY();
        heading = Math.toRadians(currentPose.getHeading());

        xVelcocity = xVelocityFilter.estimate(currentVelocity.getX());
        yVelocity = yVelocityFilter.estimate(currentVelocity.getY());
        updateGlide();

        predictedX = x + xGlide;
        predictedY = y + yGlide;


    }
}