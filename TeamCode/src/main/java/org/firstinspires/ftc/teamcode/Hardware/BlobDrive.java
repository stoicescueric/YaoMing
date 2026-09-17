package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.blob.Blob;
import com.blob.BlobParams;
import org.firstinspires.ftc.teamcode.Util.BlobConfig;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Our {@link Blob}, with the two things the in-project copy had that the library does not.
 *
 * <p><b>Pedro poses.</b> Every auto stores its waypoints as {@code com.pedropathing.math.Pose},
 * because the same constants feed Pedro. The library speaks {@code com.blob.geometry.Pose}. Rather
 * than wrapping 140-odd call sites in a converter, the overloads live here and the autos are
 * untouched.
 *
 * <p><b>Flywheel velocity.</b> The shooter encoder is wired into the {@code fl} motor's encoder
 * port, so reading flywheel speed means reading a drive motor. The old Blob exposed that as
 * {@code returnFrVelocity()}. It is robot wiring, not path following, so the library has no business
 * knowing about it and it stays here.
 */
@Config
public class BlobDrive extends Blob {

    /** Flips the sign of the flywheel reading. Which way "positive" is depends on the wiring. */
    public static double shooterSign = -1;

    private final DcMotorEx flEncoder;

    public BlobDrive(HardwareMap hardwareMap, BlobParams params, Blob.State initialState) {
        // Which odometry hardware this is comes from BlobConfig.localizerMode, so swapping a
        // Pinpoint for an OctoQuad is one enum, not a code change here.
        super(hardwareMap, params, BlobConfig.localizer(hardwareMap, params), initialState);
        // Same device the follower drives. hardwareMap hands back the same instance, so this is a
        // second reference, not a second motor.
        flEncoder = hardwareMap.get(DcMotorEx.class, params.leftFrontName);
    }

    /** Flywheel surface velocity, taken off the encoder plugged into the fl port. */
    public double returnFrVelocity() {
        return flEncoder.getVelocity() * shooterSign;
    }

    /**
     * Pedro pose to blob pose.
     *
     * <p>Replaces {@code com.blob.geometry.PedroAdapter}, which is compiled into blob's aar against
     * Pedro 2's {@code com.pedropathing.geometry.Pose}. That class does not exist in Pedro 3, so
     * calling the adapter would fail to resolve. Converting here keeps blob's aar untouched and
     * leaves that one stale class simply unreferenced.
     */
    private static com.blob.geometry.Pose toBlob(Pose p) {
        return new com.blob.geometry.Pose(p.x(), p.y(), p.heading());
    }

    public void setTargetPosition(Pose target) {
        setTargetPosition(toBlob(target));
    }

    public void setTargetPosition(Pose target, double headingThresholdPercent) {
        setTargetPosition(toBlob(target), headingThresholdPercent);
    }

    /** Curves through {@code intercept} on the way to {@code target}. */
    public void setTargetPosition(Pose target, Pose intercept) {
        setTargetPosition(toBlob(target), toBlob(intercept));
    }

    public void setTargetPosition(Pose target, Pose intercept, double headingThresholdPercent) {
        setTargetPosition(toBlob(target), toBlob(intercept),
                headingThresholdPercent);
    }

    /** Convenience for the autos, which all seed the pose from a Pedro constant. */
    public void setPose(Pose pose) {
        odo.setPose(toBlob(pose));
    }

    /** One smooth path through every waypoint, from Pedro poses. */
    public void setPath(Pose... waypoints) {
        com.blob.geometry.Pose[] converted = new com.blob.geometry.Pose[waypoints.length];
        for (int i = 0; i < waypoints.length; i++) {
            converted[i] = toBlob(waypoints[i]);
        }
        setPath(converted);
    }
}
