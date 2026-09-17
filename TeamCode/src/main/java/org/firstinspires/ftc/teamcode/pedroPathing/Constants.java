package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Pedro 3 follower setup.
 *
 * <p>Pedro 3 dropped {@code FollowerBuilder}. A follower is now composed from three pieces that are
 * built separately and handed to its constructor:
 * <pre>
 *   new Follower(Localizer, Drivetrain, Algorithm)
 * </pre>
 * and each piece is configured by a lambda rather than a fluent builder, because
 * {@link com.pedropathing.config.Configuration} is a functional interface over a bag of
 * {@link com.pedropathing.config.ConfigVar} fields.
 *
 * <h3>Values carried over from the Pedro 2 setup</h3>
 * Everything with a clean equivalent was brought across: pod offsets, motor names and directions,
 * the achievable velocities, the zero-power decelerations, and the translational and heading gains.
 *
 * <h3>Values that did NOT carry over, and why</h3>
 * <ul>
 *   <li><b>drivePIDFCoefficients</b> has no counterpart. Foresight brakes and coasts through
 *       separate controllers with their own coefficient matrices, so the old drive PIDF describes a
 *       loop that no longer exists.</li>
 *   <li><b>mass</b> and <b>centripetalScaling</b> are gone from the config entirely.</li>
 *   <li>The old {@code PathConstraints(0.99, 100, 1.4, 1)} is split across four separate
 *       constraints below. The mapping of its positional arguments is inferred and worth
 *       confirming before trusting it.</li>
 * </ul>
 *
 * <p>So this is a starting point, not a tuned follower. The gains that did carry over were tuned
 * against a different control law and should be re-tuned; that is what the autotune OpMode is for.
 *
 * <p><b>Note that nothing on this robot drives through Pedro.</b> {@code BlobDrive} follows paths,
 * and every auto commands it. This follower exists to be tuned and evaluated.
 */
@Config
public class Constants {

    // ---- Localizer: goBILDA Pinpoint ----
    public static String PINPOINT_NAME = "pinpoint";
    /**
     * Offsets in inches, carried from the Pedro 2 {@code forwardPodY} and {@code strafePodX}.
     * Pedro 3 names them by the pod rather than by the axis the offset lies on, so the mapping is
     * forward pod to X and strafe pod to Y. Check this on the robot: a swapped or sign-flipped
     * offset shows up as the pose spiralling when the robot turns in place.
     */
    public static double X_POD_OFFSET = -3.4842519685;
    public static double Y_POD_OFFSET = 4.3031496063;

    // ---- Drivetrain: mecanum ----
    public static String FRONT_LEFT = "fl", BACK_LEFT = "bl", FRONT_RIGHT = "fr", BACK_RIGHT = "br";
    public static boolean BRAKE_MODE = true;

    // ---- Foresight gains ----
    /** Old translationalPIDFCoefficients(0.078, 0, 0.0035, 0.009). Pedro 3 splits the two axes. */
    public static double TRANS_P = 0.078, TRANS_I = 0, TRANS_D = 0.0035, TRANS_F = 0.009;
    /** Old headingPIDFCoefficients(1.7, 0, 0.07, 0.02); the F term becomes a static feedforward. */
    public static double HEADING_P = 1.7, HEADING_I = 0, HEADING_D = 0.07, HEADING_F = 0.02;

    /** Old MecanumConstants.xVelocity / yVelocity, in/s. */
    public static double MAX_FORWARD_VELOCITY = 64.11;
    public static double MAX_STRAFE_VELOCITY = (51.63 + 52.08 + 50.1) / 3;
    /** Old forward/lateral zeroPowerAcceleration, in/s^2. Negative: they are decelerations. */
    public static double NATURAL_FORWARD_DECEL = (-29.20 - 30.72 - 31.99) / 3;
    public static double NATURAL_STRAFE_DECEL = (-60.06 - 54.97 - 55.97) / 3;

    /** Old PathConstraints(0.99, 100, 1.4, 1), split out. Argument mapping inferred. */
    public static double PARAMETRIC_T_CONSTRAINT = 0.99;
    public static double TIMEOUT_CONSTRAINT = 100;
    public static double TRANSLATIONAL_CONSTRAINT = 1.4;
    public static double HEADING_CONSTRAINT = 1;

    public static Localizer createLocalizer(HardwareMap hardwareMap) {
        return new PinpointLocalizer(hardwareMap, new PinpointConfig(cfg -> {
            cfg.name.set(PINPOINT_NAME);
            cfg.xPodOffset.set(X_POD_OFFSET);
            cfg.yPodOffset.set(Y_POD_OFFSET);
            cfg.offsetUnits.set(DistanceUnit.INCH);
            cfg.globalDistanceUnit.set(DistanceUnit.INCH);
            cfg.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            cfg.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            cfg.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        }));
    }

    public static Drivetrain createDrivetrain(HardwareMap hardwareMap) {
        return new Mecanum(hardwareMap, new MecanumConfig(cfg -> {
            cfg.frontLeftName.set(FRONT_LEFT);
            cfg.backLeftName.set(BACK_LEFT);
            cfg.frontRightName.set(FRONT_RIGHT);
            cfg.backRightName.set(BACK_RIGHT);
            cfg.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            cfg.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            cfg.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
            cfg.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
            cfg.manualBrakeMode.set(BRAKE_MODE);
        }));
    }

    public static Algorithm createAlgorithm() {
        return new Foresight(new ForesightConfig(cfg -> {
            // The old PIDF F term is a static feedforward in Pedro 3, summed onto the PID.
            cfg.forwardTranslational.set(Controller.sum(
                    Controller.pid(TRANS_P, TRANS_I, TRANS_D),
                    Controller.staticFeedforward(TRANS_F)));
            cfg.strafeTranslational.set(Controller.sum(
                    Controller.pid(TRANS_P, TRANS_I, TRANS_D),
                    Controller.staticFeedforward(TRANS_F)));
            cfg.headingFeedback.set(Controller.pid(HEADING_P, HEADING_I, HEADING_D));
            cfg.headingStaticFF.set(Controller.staticFeedforward(HEADING_F));

            cfg.maxAchievableForwardVelocity.set(MAX_FORWARD_VELOCITY);
            cfg.maxAchievableStrafeVelocity.set(MAX_STRAFE_VELOCITY);
            cfg.naturalForwardDeceleration.set(NATURAL_FORWARD_DECEL);
            cfg.naturalStrafeDeceleration.set(NATURAL_STRAFE_DECEL);

            cfg.parametricTConstraint.set(PARAMETRIC_T_CONSTRAINT);
            cfg.timeoutConstraint.set(TIMEOUT_CONSTRAINT);
            cfg.translationalConstraint.set(TRANSLATIONAL_CONSTRAINT);
            cfg.headingConstraint.set(HEADING_CONSTRAINT);
        }));
    }

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(createLocalizer(hardwareMap), createDrivetrain(hardwareMap),
                createAlgorithm());
    }
}
