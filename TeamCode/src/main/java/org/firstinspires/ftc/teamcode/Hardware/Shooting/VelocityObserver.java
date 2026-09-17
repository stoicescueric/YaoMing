package org.firstinspires.ftc.teamcode.Hardware.Shooting;

import com.acmerobotics.dashboard.config.Config;
import com.blob.control.DriveFeedforward;

/**
 * A smoother, less laggy acceleration estimate than differentiated odometry.
 *
 * <p>The measured signal is poor: blob differences an already-filtered velocity at loop rate, which
 * multiplies the OctoQuad's quantisation by 1/dt, then smooths the result with a 22 ms constant.
 * That is almost certainly why the acceleration term in the old shot maths was left multiplied by
 * zero.
 *
 * <p>The drivetrain is a measured first-order plant, {@code V = kS*sgn(v) + kV*v + kA*a}, so the
 * commanded voltage tells us the acceleration directly:
 * <pre>
 *   a = (V - kS*sgn(v) - kV*v) / kA
 * </pre>
 * That estimate is smooth and it <em>leads</em> the measurement, because a command precedes the
 * response. It is also blind to anything the model does not know about, such as wheel slip, being
 * pushed, or a stalled wheel, so it is blended with the measurement rather than replacing it.
 *
 * <p>This is the principled home for "use the stick input": the driver's command is the control
 * input to a state observer, not a separate prediction of the future.
 *
 * <p>{@link #MODEL_BLEND} defaults to 0, so nothing changes until the frame convention below has
 * been checked on the robot.
 */
@Config
public class VelocityObserver {

    /** 0 = measured acceleration only, 1 = model only. Raise once validated. */
    public static double MODEL_BLEND = 0.0;
    /** Wheel command magnitude below which the model is ignored, since kS dominates at rest. */
    public static double MIN_COMMAND = 0.05;

    /** Per-axis drivetrain constants. Fill from BlobConfig so there is one measured source. */
    public static double FWD_KS = 1.005556977145929, FWD_KV = 0.1531052616714671, FWD_KA = 0.020939451511571863;
    public static double LAT_KS = 1.8496674201113557, LAT_KV = 0.16649036357220193, LAT_KA = 0.018977932127223332;

    private final DriveFeedforward forward = new DriveFeedforward();
    private final DriveFeedforward lateral = new DriveFeedforward();

    public double modelAx, modelAy;
    public double fusedAx, fusedAy;

    /**
     * @param wheelCommand  last commands sent, unit power, [LF, LB, RF, RB] — blob's order
     * @param batteryVolts  battery voltage
     * @param heading       field heading, radians
     * @param vx            measured field velocity
     * @param vy            see vx
     * @param measuredAx    measured field acceleration
     * @param measuredAy    see measuredAx
     */
    public void update(double[] wheelCommand, double batteryVolts, double heading,
                       double vx, double vy, double measuredAx, double measuredAy) {

        fusedAx = measuredAx;
        fusedAy = measuredAy;
        if (wheelCommand == null || wheelCommand.length < 4 || MODEL_BLEND <= 0) return;

        forward.kS = FWD_KS; forward.kV = FWD_KV; forward.kA = FWD_KA;
        lateral.kS = LAT_KS; lateral.kV = LAT_KV; lateral.kA = LAT_KA;

        // Invert blob's mecanum mix. It builds LF=(y+x+rx), LB=(y-x+rx), RF=(y-x-rx), RB=(y+x-rx),
        // all over a shared denominator, so the averages below recover the normalised commands
        // without needing to know what that denominator was.
        double lf = wheelCommand[0], lb = wheelCommand[1], rf = wheelCommand[2], rb = wheelCommand[3];
        double cmdForward = (lf + lb + rf + rb) / 4.0;
        double cmdStrafe  = (lf - lb - rf + rb) / 4.0;

        if (Math.abs(cmdForward) + Math.abs(cmdStrafe) < MIN_COMMAND) return;

        // The plant constants are per robot axis, so the inversion has to happen in the robot frame.
        double cos = Math.cos(heading), sin = Math.sin(heading);
        double vFwd =  vx * cos + vy * sin;
        double vLat = -vx * sin + vy * cos;

        double aFwd = accelFrom(forward, cmdForward * batteryVolts, vFwd);
        double aLat = accelFrom(lateral, cmdStrafe * batteryVolts, vLat);

        modelAx = aFwd * cos - aLat * sin;
        modelAy = aFwd * sin + aLat * cos;

        double k = Math.max(0, Math.min(1, MODEL_BLEND));
        fusedAx = (1 - k) * measuredAx + k * modelAx;
        fusedAy = (1 - k) * measuredAy + k * modelAy;
    }

    private static double accelFrom(DriveFeedforward ff, double volts, double velocity) {
        if (ff.kA <= 1e-9) return 0;
        return (volts - ff.kS * Math.signum(velocity) - ff.kV * velocity) / ff.kA;
    }
}
