package org.firstinspires.ftc.teamcode.Util;

import com.acmerobotics.dashboard.config.Config;
import com.blob.BlobParams;
import com.blob.localization.GoBildaPinpointDriver;
import com.blob.localization.Localizer;
import com.blob.localization.OctoQuadFWv3Localizer;
import com.blob.localization.OctoQuadLocalizer;
import com.blob.localization.OctoQuadTwoWheelLocalizer;
import com.blob.localization.PinpointLocalizer;
import com.blob.localization.SdkPinpointLocalizer;
import com.blob.tuning.TuningConfig;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

/**
 * Our tuned blob numbers, and the bridge between them and the library's {@link BlobParams}.
 *
 * <p>The library takes config as an instance you hand to the constructor, while FtcDashboard can
 * only edit static fields. So the statics live here and {@link #sync} copies them into the live
 * params object every loop, which is what keeps dashboard sliders working. {@code Blob.update()}
 * re-reads the gains from params each loop anyway, so a value edited mid-run takes effect
 * immediately, exactly like the old static {@code BlobConstants} did.
 *
 * <p>Replaces {@code org.firstinspires.ftc.teamcode.blob.constants.BlobConstants}. Values carried
 * over unchanged, so behaviour is identical to the in-project follower.
 */
@Config
public class BlobConfig {

    /**
     * Which odometry hardware to localize from. Every one of these is two pods plus an IMU; they
     * differ in whose IMU it is and who fuses it.
     *
     * <p>Changing this is not enough on its own. Each mode needs a matching entry in the robot
     * config, and the OctoQuad modes need their pod geometry filled in below.
     */
    public enum LocalizerMode {
        /**
         * goBILDA Pinpoint via the SDK's own driver. Config entry: "goBILDA Pinpoint Odometry
         * Computer". This is what our robot runs, and it needs nothing changed on the hub.
         */
        PINPOINT_SDK,

        /**
         * goBILDA Pinpoint via blob's bundled driver. Config entry: "blob Pinpoint (goBILDA)".
         * Only for SDKs older than 10.2, which have no built-in Pinpoint driver.
         */
        PINPOINT_BUNDLED,

        /**
         * OctoQuad MK2 via the SDK's built-in driver. Config entry: "OctoQuadFTC".
         *
         * <p>If your board's firmware minor revision is newer than the SDK driver expects, this
         * puts a permanent warning banner on the Driver Station even though nothing is wrong. Use
         * {@link #OCTOQUAD_MK2_FWV3} instead in that case.
         */
        OCTOQUAD_MK2,

        /**
         * OctoQuad MK2 via blob's vendored copy of DigitalChickenLabs' firmware v3 driver.
         * Config entry: "blob OctoQuad (FW v3)". This is what our robot runs.
         *
         * <p>Same board, same behaviour as {@link #OCTOQUAD_MK2}. The only difference is that a
         * newer-minor firmware revision is logged rather than raised as a Driver Station banner,
         * because a newer minor revision is backward compatible and nothing is actually broken.
         */
        OCTOQUAD_MK2_FWV3,

        /**
         * Any OctoQuad, including MK1, with heading from the Control Hub IMU. blob integrates the
         * pods itself. Needs an "imu" in the config, the hub orientation, and the pod offsets in
         * INCHES.
         */
        OCTOQUAD_TWO_WHEEL
    }

    /** Which of the above to build. The only line to change when swapping odometry hardware. */
    public static LocalizerMode localizerMode = LocalizerMode.OCTOQUAD_MK2_FWV3;

    // HardwareMap
    public static String leftFrontName = "fl";
    public static String leftBackName = "bl";
    public static String rightFrontName = "fr";
    public static String rightBackName = "br";
    public static String pinpointName = "pinpoint";

    // In-position thresholds
    public static double hDefTresh = Math.toRadians(3);
    public static double xDefTresh = 1;
    public static double yDefTresh = 1;
    public static double defaultVelocityThresh = 4;
    public static double defaultTransThresh = 2;

    // PID. kI and hI are ignored: Blob.update() forces the integral term to zero.
    public static double kP = 0.05, kI = 0, kD = 0.006;
    public static double hP = 1.2, hI = 0, hD = 0.11;

    // Deceleration, from the deceleration tuner
    // Measured by autotune, from the stop at the end of every translational run. Two runs agreed
    // to within four percent, which is better than most things here. They were 80 and 90 before,
    // library defaults nobody had checked, and the RST profile brakes on them: too low and it
    // creeps in from further out than it needs to.
    //
    // Measured with the volts off and the motors braking, so a floor rather than the hardest this
    // drivetrain could stop under power. Do not raise them by hand.
    // NOT the measured figures, on purpose. Autotune measured 116 and 125, and those are the most
    // this drivetrain can do: at that rate the wheels are at the edge of grip. The profile brakes on
    // these numbers, so planning every stop at the limit means arriving at every stop with no grip
    // left over, and a wheel that is slipping cannot hold heading. That is the wobble.
    //
    // About 73% of measured, which leaves the rest for steering while it stops. 65% held the
    // heading but gave away more of the approach than it needed to.
    // Measured by blob: Autotune, from the stop at the end of every translational run.
    //
    // These are a FLOOR, not the limit: they come from braking with the volts off and the motors
    // shorted, which is less than the drivetrain could manage under power. A profile built on a
    // floor brakes early rather than late, which is the safe direction. Do not raise them by hand.
    public static double xDeceleration = 103.36192668078725;
    public static double yDeceleration = 130.07101355727193;

    /**
     * Cap on the rotational command only, 0 to 1. Was exposed on the old robot's branch and is kept.
     *
     * <p>Lower it if the robot turns faster than the gyro can measure: past the sensor's range the
     * reported rate stops rising with the real one, so rotation is under-counted for as long as the
     * spin lasts and no heading scalar recovers it. blob: Heading Scalar Tuner reports the peak rate
     * it saw, which is how you find out.
     */
    public static double maxTurnPower = 1.0;

    public static double lateralMultiplier = 1.4;
    public static double voltageConstant = 12;

    // Localization: OctoQuad MK2.
    //
    // These are the measured offsets for this chassis, in PINPOINT convention: pod offsets in
    // inches. They happen to coincide with blob's library defaults, which is a coincidence and not
    // a sign they were never set.
    //
    //   xOffset  the FORWARD pod, inches LEFT of the centre of rotation
    //   yOffset  the STRAFE pod, inches AHEAD of the centre of rotation
    //
    // This is the only place either number appears. sync() derives the board's TCP offsets with
    // octoQuadOffsetsFromPinpoint():
    //
    //     octoQuadTcpOffsetMM_X = -yOffset * 25.4
    //     octoQuadTcpOffsetMM_Y = -xOffset * 25.4
    //
    // The axes swap AND both signs flip, so it is not a unit conversion and not worth doing by
    // hand. Deriving rather than storing is the point: the config used to set the millimetre values
    // directly and ALSO derive them, so editing either one silently lost.
    //
    // Worth re-checking on a new chassis even though the numbers carried over: blob: Rotation
    // Offset Tuner, turning the robot BY HAND. Spin in place and the reported position should stay
    // a dot rather than trace a circle. Its correction is in these same inches.
    public static double xOffset = -3.1517784347744198;
    public static double yOffset = 4.4790201071404985;

    public static GoBildaPinpointDriver.EncoderDirection xPodDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection yPodDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    // Which pods the new robot actually has. 4-bar is 19.89 counts/mm, swingarm is 13.26, and
    // getting this wrong scales every distance without looking obviously broken.
    public static GoBildaPinpointDriver.GoBildaOdometryPods podType = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    /**
     * Aims the PID at the glide-predicted position instead of the current one. Was a static on the
     * old Blob; it lives on the instance now, so Robot.update() pushes it across.
     */
    public static boolean usePredictiveDecel = false;

    // OctoQuad geometry. Ignored unless localizerMode is one of the OCTOQUAD_ ones.
    public static String octoQuadName = "octoquad";
    public static int octoQuadPortX = 7;   // forward-reading pod
    public static int octoQuadPortY = 0;   // strafe-reading pod
    /**
     * Counts per mm of pod travel. The Quickstart guide recommends MEASURING this by pushing the
     * robot a known distance rather than trusting the spec figure, because it depends on the wheel
     * as built, not on paper. This is the goBILDA 4-bar spec value; swingarm is 13.26291192.
     */
    public static double octoQuadCountsPerMM = 19.89436789;

    /**
     * Pod directions. Required: with the robot at 0 degrees, pushing it FORWARD must make the X pod
     * count up, and pushing it LEFT must make the Y pod count up. Expect to flip at least one.
     */
    public static GoBildaPinpointDriver.EncoderDirection octoQuadXDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    public static GoBildaPinpointDriver.EncoderDirection octoQuadYDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;

    /**
     * Corrects the IMU reporting more or less rotation than actually happened. A wrong scalar does
     * not cause drift; it makes every turn come out the wrong size, which shows up as heading
     * skewing further and further out the more the robot spins.
     *
     * <p><b>Still a placeholder, not a measurement.</b> Run {@code blob: Heading Scalar Tuner},
     * turn the robot by hand through ten full turns, and put the number it gives you here.
     */
    public static double octoQuadImuHeadingScalar = 1.0221;

    /** Hardware velocity averaging window, ms. Longer is smoother with more lag. */
    public static int octoQuadVelocityIntervalMS = 25;

    /**
     * How long to wait for the MK2 IMU to finish calibrating at init, ms.
     *
     * <p>Matches the library default. Giving up early means using the IMU before its zero-rate bias
     * is calibrated, and that bias is what stops heading drifting. Note this value is synced into
     * the params every loop, so it wins over whatever the library ships.
     */
    public static long octoQuadInitTimeoutMs = 5000;

    public static boolean octoQuadInvertHeading = false;

    /** Two-wheel mode only, so unused here. Counts per INCH rather than per mm. */
    public static double octoQuadTicksPerInch = 19.89436789 * 25.4;

    // -----------------------------------------------------------------------------------
    // Measured drivetrain constants, from blob: Autotune. Paste its output here.
    //
    // All zero means not measured, and Blob.State.RST refuses to drive rather than following
    // a model of nothing. State.PID ignores them entirely, so leaving them empty costs nothing.
    // -----------------------------------------------------------------------------------
    // The first run measured up to the speeds the follower actually uses, with an acceleration
    // that belongs to the sample it is attached to rather than lagging it. Both mattered.
    //
    // It disagrees with the earlier runs in a way that explains the overshoot at speed. They only
    // ever saw 41 in/s and the straight line they fitted through that says holding 60 in/s costs
    // 9.46V. This one went to 63 and says it costs 10.97, sixteen percent more, so the drivetrain
    // is not quite first order out there and the earlier fits were extrapolating into exactly the
    // region the follower spends its time in. They also promised a top speed of 71 in/s where the
    // real one is 60.
    // Measured on this robot by blob: Autotune. Lane use was 38.7 in along and 2.29 in off, both
    // inside the envelope, so the runs stayed where they were supposed to.
    //
    //   forward  top 76.0 in/s   tau 0.145s
    //   lateral  top 65.1 in/s   tau 0.121s
    //   angular  top 8.82 rad/s (506 deg/s)   tau 0.087s
    //
    // Note the strafe penalty is almost entirely FRICTION, not back-EMF: lateral kV is only 1.04x
    // forward while lateral kS is 2.51x. lateralMultiplier models it as a kV ratio, so on the PID
    // path it corrects for something that is barely there and misses the 1.3V that actually costs.
    // State.RST handles both terms separately and ignores lateralMultiplier entirely.
    public static double forwardKS = 0.8660081949863851;
    public static double forwardKV = 0.15432954439427538;
    public static double forwardKA = 0.02238380856962927;

    public static double lateralKS = 2.173309217171191;
    public static double lateralKV = 0.1601648028312494;
    public static double lateralKA = 0.01940715978169199;

    public static double angularKS = 1.4695597700047904;
    public static double angularKV = 1.2614630240789948;
    public static double angularKA = 0.10958105457960919;

    /**
     * How fast an RST velocity loop should converge, seconds.
     *
     * <p>0.08 is the four-loop FLOOR, not anything about this drivetrain: half the slowest axis tau
     * would be 0.0725, and the autotune floors at four periods of an assumed 20 ms loop. This robot
     * runs nearer 60 Hz, where four periods is 0.067, so 0.0725 is reachable. Worth trying once the
     * follower is behaving, for slightly quicker tracking.
     */
    public static double rstTauClosedLoop = 0.08;
    /**
     * Slowed from 0.394. The integrator is what stores a correction that did not take effect, and
     * under braking that happens: the wheels slip, the heading does not come back, the integrator
     * winds, and when grip returns the stored command arrives all at once.
     */
    public static double rstTauIntegral = 0.4;
    /**
     * Final approach time constant, seconds. Larger settles gentler, smaller arrives sooner.
     *
     * <p>Raised from 0.25 along with the deceleration margin: the last few inches are where the
     * heading has to settle, and it cannot do that while the translation loop is still asking for
     * everything the wheels have.
     */
    public static double rstApproachTau = 0.30;
    /**
     * How fast a turn to ask for, as opposed to how fast to meet it.
     *
     * <p>Raised from 0.3 because the heading wobbled while braking. Braking is when the wheels are
     * closest to slipping, so a heading correction asked for then is the one least likely to take
     * effect, and the loop answers by asking for more. Half a second is still a brisk correction:
     * ten degrees out asks for 0.35 rad/s, closed in about half a second.
     */
    public static double rstHeadingTau = 0.5;

    /**
     * Reference speed over which the whole kS is applied. Below it the term scales down.
     *
     * <p>Turn these DOWN for more push out of corners, UP if the robot wiggles as it settles: too
     * small and the term stops being a ramp and becomes a switch, reversing a couple of volts every
     * time the robot crosses the line it is following.
     */
    public static double rstStaticRampSpeed = 8.0;

    /**
     * Raised from 0.8 for the same reason. At 0.8 a heading error of 14 degrees already drew the
     * whole angular kS; at 2.0 the term stays proportional through everything a settling heading
     * does and only reaches full value on a real commanded turn.
     */
    public static double rstStaticRampOmega = 2.0;

    /** Inside this and slow, the follower stops commanding instead of hunting. 0 holds position. */
    public static double rstSettleTolerance = 1.0;

    /**
     * Turn rate held in reserve while travelling, rad/s. This is the wobble knob.
     *
     * <p>Cruising costs kS + kV*v, and on this robot that is most of the pack. Spending all of it on
     * going forwards left the turn axis 1.7V, of which 1.3V is its own breakaway: 14 deg/s of
     * authority at speed. The heading loop was starved, not badly tuned, which is why softening its
     * gains did nothing.
     *
     * <p>What it buys on THIS robot, from its own measured constants:
     *
     * <pre>
     *   0.00 rad/s   cruise 68.4 in/s   heading   0 deg/s
     *   0.50 rad/s   cruise 56.2 in/s   heading  72 deg/s
     *   0.75 rad/s   cruise 54.3 in/s   heading  85 deg/s
     *   1.00 rad/s   cruise 52.5 in/s   heading  98 deg/s
     * </pre>
     *
     * <p>A better trade than the old chassis got: this one turns more easily, angular kV being 1.26
     * against 1.95, so the same reserve buys 72 deg/s here where it bought 56 there, and the speed
     * given up between 0.5 and 1.0 is only 4 in/s.
     *
     * <p>0.5 to start. Raise it if it still wobbles, lower it for speed once it does not.
     */
    public static double rstTurnAuthority = 0.5;

    /**
     * How much of the measured lateral grip a corner may use, 0 to 1.
     *
     * <p>The follower caps its speed at sqrt(grip / curvature), so this decides how hard it corners.
     * The grip figure comes from braking in a straight line, and a wheel near its limit one way has
     * less to give another, hence a fraction rather than all of it. Zero turns the limit off.
     */
    public static double rstCurvatureGripFraction = 0.8;

    /**
     * How much of the computed centripetal feedforward to apply.
     *
     * <p>1 is the model's own answer: kA volts per unit of the v^2 * curvature the corner needs. Not
     * a gain to search for. Lower it if the robot cuts inside a curve, raise it if it runs wide.
     */
    public static double rstCentripetalScale = 1.0;

    /**
     * Speed cap for the RST follower, in/s. 0 would derive it from the constants and the battery.
     *
     * <p>Zero, so it follows the constants and the battery rather than a number that has to be
     * revisited every time either changes. On this drivetrain that works out near 59 in/s: the top
     * speed is 70, and rstSpeedFraction leaves the rest as headroom for the loop to correct with.
     *
     * <p>The thing to watch is the stop, not the speed. The profile brakes on xDeceleration and
     * yDeceleration, and those are still the library's defaults rather than anything measured here.
     * At 80 in/s^2 a stop from 70 needs 31 inches; if the real figure is nearer 50 it needs 49, and
     * the difference arrives as overshoot at a pickup pose. The motors are not the limit either way,
     * they can brake at over a thousand, so what those numbers describe is the carpet.
     *
     * <p>blob: Deceleration Tuner measures them. It is the one input the autotune does not produce.
     */
    public static double rstMaxVelocity = 0;

    /** Turn rate cap, rad/s. 0 derives it from the constants and the battery. */
    public static double rstMaxAngularVelocity = 0;
    /** How much of the achievable speed to use. Translational headroom only; turn is reserved. */
    public static double rstSpeedFraction = 0.9;

    // Curves
    public static double splineLookahead = 6.0;
    public static int splineSamples = 64;

    // Heading modes. Only read when headingMode is something other than FIXED.
    /**
     * Fraction of the leg at which a tangent heading stops following the direction of travel and
     * starts rotating onto the target pose's own heading. 1 stays tangential all the way.
     */
    public static double tangentSettleFraction = 0.8;
    /** Inches. Below this the aim vector is too short for its angle to be trusted. */
    public static double tangentMinDistance = 2.0;

    /**
     * Records paths for {@code pusher visualiser}. Only does anything on the dev artifact; the
     * competition one has no recorder to switch on.
     */
    public static boolean recordTrace = false;

    /**
     * Logs every stage of every follower loop to /sdcard/FIRST/blob-follower.
     *
     * <p>For telling a mechanical wobble from a software one. Pull the CSV and run
     * tools/replay-follower.py on it: if the recorded inputs reproduce the recorded outputs, the
     * code does what we think and the fault is in the robot. Dev artifact only, a row per loop, so
     * turn it off when you are done.
     */
    public static boolean recordFollower = true;

    /** Builds the localizer for {@link #localizerMode}. */
    public static Localizer localizer(HardwareMap hardwareMap, BlobParams params) {
        switch (localizerMode) {
            case PINPOINT_BUNDLED:
                return new PinpointLocalizer(hardwareMap, params);
            case OCTOQUAD_MK2:
                return new OctoQuadLocalizer(hardwareMap, params);
            case OCTOQUAD_MK2_FWV3:
                return new OctoQuadFWv3Localizer(hardwareMap, params);
            case OCTOQUAD_TWO_WHEEL:
                return new OctoQuadTwoWheelLocalizer(hardwareMap, params);
            case PINPOINT_SDK:
            default:
                return new SdkPinpointLocalizer(hardwareMap, params);
        }
    }

    /** A fresh params object with everything above applied. */
    public static BlobParams params() {
        BlobParams p = new BlobParams();
        sync(p);
        return p;
    }

    /** Pushes the current static values into an existing params object. Call once per loop. */
    public static void sync(BlobParams p) {
        p.leftFrontName = leftFrontName;
        p.leftBackName = leftBackName;
        p.rightFrontName = rightFrontName;
        p.rightBackName = rightBackName;
        p.pinpointName = pinpointName;

        p.headingThreshRad = hDefTresh;
        p.xThresh = xDefTresh;
        p.yThresh = yDefTresh;
        p.defaultVelocityThresh = defaultVelocityThresh;
        p.defaultTransThresh = defaultTransThresh;

        p.kP = kP; p.kI = kI; p.kD = kD;
        p.hP = hP; p.hI = hI; p.hD = hD;

        p.forwardKS = forwardKS; p.forwardKV = forwardKV; p.forwardKA = forwardKA;
        p.lateralKS = lateralKS; p.lateralKV = lateralKV; p.lateralKA = lateralKA;
        p.angularKS = angularKS; p.angularKV = angularKV; p.angularKA = angularKA;
        p.rstTauClosedLoop = rstTauClosedLoop;
        p.rstTauIntegral = rstTauIntegral;
        p.rstApproachTau = rstApproachTau;
        p.rstHeadingTau = rstHeadingTau;
        p.rstStaticRampSpeed = rstStaticRampSpeed;
        p.rstStaticRampOmega = rstStaticRampOmega;
        p.rstSettleTolerance = rstSettleTolerance;
        p.rstMaxVelocity = rstMaxVelocity;
        p.rstMaxAngularVelocity = rstMaxAngularVelocity;
        p.rstTurnAuthority = rstTurnAuthority;
        p.rstCurvatureGripFraction = rstCurvatureGripFraction;
        p.rstCentripetalScale = rstCentripetalScale;
        p.rstSpeedFraction = rstSpeedFraction;

        p.xDeceleration = xDeceleration;
        p.yDeceleration = yDeceleration;
        p.maxTurnPower = maxTurnPower;
        p.lateralMultiplier = lateralMultiplier;

        p.xOffset = xOffset;
        p.yOffset = yOffset;
        p.xPodDirection = xPodDirection;
        p.yPodDirection = yPodDirection;
        p.podType = podType;

        p.octoQuadName = octoQuadName;
        p.octoQuadPortX = octoQuadPortX;
        p.octoQuadPortY = octoQuadPortY;
        p.octoQuadCountsPerMM_X = octoQuadCountsPerMM;
        p.octoQuadCountsPerMM_Y = octoQuadCountsPerMM;
        p.octoQuadXDirection = octoQuadXDirection;
        p.octoQuadYDirection = octoQuadYDirection;
        p.octoQuadImuHeadingScalar = octoQuadImuHeadingScalar;
        p.octoQuadVelocityIntervalMS = octoQuadVelocityIntervalMS;
        p.octoQuadInitTimeoutMs = octoQuadInitTimeoutMs;
        p.octoQuadInvertHeading = octoQuadInvertHeading;
        // Set directly rather than derived from xOffset/yOffset. One source of truth, in the units
        // the board actually takes.
        // Derived from the inch offsets above rather than set, so there is one place to edit.
        p.octoQuadOffsetsFromPinpoint();
        p.octoQuadTicksPerInch = octoQuadTicksPerInch;
        // The two-wheel localizer reuses the Pinpoint's pod geometry, so there is one set to tune.
        p.parallelPodYOffset = xOffset;
        p.perpendicularPodXOffset = yOffset;

        p.splineLookahead = splineLookahead;
        p.splineSamples = splineSamples;
        p.tangentSettleFraction = tangentSettleFraction;
        p.tangentMinDistance = tangentMinDistance;
        p.recordTrace = recordTrace;
        p.recordFollower = recordFollower;
    }

    /**
     * Points the library's bundled tuning OpModes at these values instead of stock defaults.
     * Without this they would tune against a robot that is not ours.
     */
    @OpModeRegistrar
    public static void registerTuning(OpModeManager manager) {
        TuningConfig.useParams(BlobConfig::params);
        TuningConfig.useLocalizer(BlobConfig::localizer);
    }
}
