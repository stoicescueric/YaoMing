package org.firstinspires.ftc.teamcode.Hardware.Shooting;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Util.Wrapper.InterpLUT;

/**
 * Everything about how the ball flies: how long it is in the air, and how much of the robot's
 * velocity it keeps once it is.
 *
 * <p>Deliberately separate from the solver. The solver is geometry and does not change when the ball
 * does; this class is the part that has to be re-measured for a new ball, and it is the only place
 * that knows about drag.
 *
 * <h3>Why the lead factor is not 1</h3>
 * In a vacuum, aiming at {@code goal - v*T} is exact: adding horizontal velocity to a projectile
 * changes its horizontal displacement by {@code v*T} and leaves the vertical motion, and therefore
 * the flight time, untouched. Drag breaks that, because it acts on speed magnitude and couples the
 * axes. The inherited velocity decays in flight, so the real displacement is
 * <pre>
 *   v * tau_d * (1 - exp(-T / tau_d))
 * </pre>
 * which is {@code v * T * factor} with {@code factor < 1}. {@link #DRAG_TAU} is that {@code tau_d}.
 *
 * <p>The factor falls as flight time rises, so it cannot be a constant across a range of distances
 * the way the old {@code weightTOF} was. It also falls as the ball gets smaller: tau_d scales with
 * diameter, so halving the ball roughly halves tau_d and pushes the factor down noticeably.
 *
 * <p>{@link #USE_DRAG_MODEL} is false by default, which reproduces the old constant-factor
 * behaviour so the current robot is unchanged until the constants are measured.
 */
@Config
public class BallisticsModel {

    // Constant-factor fallbacks, the old weightTOF pair. Kept for comparison only: with the ball
    // measured, the physical lateral factor is 0.87-0.91, so 0.70 was never drag. It sits there
    // because SOTM_GAIN = 1.30 multiplied it, and 0.70 * 1.30 recovers the physical answer to
    // within 3 percent. Two knobs for one degree of freedom, cancelling each other out.
    /** Constant-factor fallback, the old {@code weightTOF}. Used when {@link #USE_DRAG_MODEL} is off. */
    public static double LATERAL_FACTOR = 0.7;
    /** Constant-factor fallback for the radial axis, the old {@code weightTOFRadial}. */
    public static double RADIAL_FACTOR = 0.55;

    /**
     * Uses the drag model rather than the two constants above. On by default: with the real ball
     * measured the model is grounded and the constants are not. Only has an effect once the solver
     * is driving, so this changes nothing on its own.
     */
    public static boolean USE_DRAG_MODEL = true;
    /**
     * Aerodynamic time constant of the ball in flight, seconds.
     *
     * <p>Computed, not fitted: {@code 2m / (rho*Cd*A*v)} for the DECODE ball (5 in, 0.078 kg, so
     * m/A = 6.16 kg/m^2) at a representative 200 in/s gives 4.21 s. It varies with flight speed
     * (4.8 s at 176 in/s down to 3.2 s at 265 in/s), but the lead factor that comes out only moves
     * between 0.91 and 0.87 across that band, so one constant is enough.
     *
     * <p>Recompute with {@link #refreshDragTau} if the ball changes. tau scales with m/A, so a ball
     * of half the diameter at the same density halves it.
     */
    public static double DRAG_TAU = 4.2;

    /** DECODE ball diameter, metres. 5 inches. */
    public static double BALL_DIAMETER_M = 0.127;
    /** DECODE ball mass, kg. */
    public static double BALL_MASS_KG = 0.078;

    /** Recomputes {@link #DRAG_TAU} from the ball constants at a given flight speed, in/s. */
    public static void refreshDragTau(double flightSpeedInPerSec) {
        DRAG_TAU = dragTauFromBall(BALL_MASS_KG, BALL_DIAMETER_M, flightSpeedInPerSec);
    }

    /** Pins the flight time instead of reading the table. The old {@code useFixedTof}. */
    public static boolean USE_FIXED_TOF = false;
    public static double FIXED_TOF = 0.6;

    private InterpLUT flightTime = new InterpLUT();

    public BallisticsModel() {
        rebuild(DEFAULT_DISTANCES, DEFAULT_TIMES);
    }

    /** Distances, inches. Paired with {@link #DEFAULT_TIMES}. */
    public static double[] DEFAULT_DISTANCES = {50, 60, 76, 85, 95, 110, 130};
    /** Flight times, seconds, for the DECODE ball. Re-measure for a new ball. */
    public static double[] DEFAULT_TIMES = {0.5, 0.54, 0.6, 0.65, 0.7, 0.8, 0.9};

    /** Rebuilds the flight-time table. Call after editing the arrays from the dashboard. */
    public void rebuild(double[] distances, double[] times) {
        InterpLUT lut = new InterpLUT();
        int n = Math.min(distances.length, times.length);
        for (int i = 0; i < n; i++) lut.add(distances[i], times[i]);
        lut.createLUT();
        flightTime = lut;
    }

    /**
     * Flight time for a distance, seconds. The table is built in safe mode, so a distance outside
     * it clamps to the nearest end rather than throwing.
     */
    public double timeOfFlight(double distanceInches) {
        if (USE_FIXED_TOF) return FIXED_TOF;
        try {
            return flightTime.get(distanceInches);
        } catch (Exception e) {
            return FIXED_TOF;
        }
    }

    /**
     * Fraction of {@code v * T} the ball actually carries sideways. Multiplies the lateral lead.
     */
    public double lateralLeadFactor(double timeOfFlight) {
        return USE_DRAG_MODEL ? decayFactor(timeOfFlight) : LATERAL_FACTOR;
    }

    /** As {@link #lateralLeadFactor}, for the radial axis, which drives range rather than azimuth. */
    public double radialLeadFactor(double timeOfFlight) {
        return USE_DRAG_MODEL ? decayFactor(timeOfFlight) : RADIAL_FACTOR;
    }

    // ---- exit velocity ----
    //
    // Measured fit: v_exit = EXIT_V_PER_TPS * tps + EXIT_V_OFFSET, inches/sec.
    //
    // NOT a model through the origin: at tps = 0 it returns EXIT_V_OFFSET, which is nonsense. It is
    // a local linearisation over the band that was actually measured, so do not extrapolate far
    // below it. The current velocity LUT spans 1320..2250 tps, i.e. about 176..265 in/s.
    /** Inches/sec of exit velocity per tick/sec of flywheel. */
    public static double EXIT_V_PER_TPS = 0.0963;
    /** Intercept of the exit-velocity fit, in/s. Not physical on its own; see above. */
    public static double EXIT_V_OFFSET = 48.6;

    /** Ball exit velocity for a flywheel speed, in/s. */
    public static double exitVelocity(double tps) {
        return EXIT_V_PER_TPS * tps + EXIT_V_OFFSET;
    }

    /** Flywheel speed needed for an exit velocity, ticks/sec. Inverse of {@link #exitVelocity}. */
    public static double tpsForExitVelocity(double exitVelocity) {
        if (Math.abs(EXIT_V_PER_TPS) < 1e-9) return 0;
        return (exitVelocity - EXIT_V_OFFSET) / EXIT_V_PER_TPS;
    }

    /**
     * Launch elevation implied by a shot, degrees. Diagnostic only, nothing reads it.
     *
     * <p>Cross-checks three independently measured things against each other: the flight-time table,
     * the velocity table and the exit-velocity fit. The mean horizontal speed a shot achieved is
     * {@code distance / tof}, and that has to be {@code exitVelocity * cos(elevation)}, so the
     * elevation falls out. On the DECODE numbers this lands at 48 to 55 degrees over the whole
     * range, which is the sign that all four calibrations describe one real shooter.
     *
     * <p>It reads high, because {@code distance / tof} is the <em>mean</em> horizontal speed and drag
     * bleeds the launch value down over the flight. Use it for consistency, not for absolute truth:
     * a kink in this curve means one of the three tables disagrees with the others near there.
     *
     * @return degrees, or NaN when the numbers are inconsistent (mean speed above exit speed)
     */
    public static double impliedLaunchElevationDeg(double distance, double timeOfFlight, double tps) {
        if (timeOfFlight <= 1e-6) return Double.NaN;
        double ve = exitVelocity(tps);
        if (ve <= 1e-6) return Double.NaN;
        double cos = (distance / timeOfFlight) / ve;
        if (cos > 1 || cos < -1) return Double.NaN;
        return Math.toDegrees(Math.acos(cos));
    }

    // ---- drag, from physical ball properties ----
    //
    // Cd = 0.47 is the sphere value and it is a QUADRATIC coefficient, so drag deceleration is
    // (rho*Cd*A/2m)*|v|*v. Linearising the lateral component about the flight speed gives the decay
    // time constant this class uses:
    //
    //     tau_d = 2m / (rho * Cd * A * v),    A = pi d^2 / 4
    //
    // tau_d is therefore a property of the BALL (mass over frontal area) and the speed it flies at,
    // not a free parameter. Compute it with dragTauFromBall rather than fitting it blind: a fitted
    // value that disagrees with the physical one means whatever LATERAL_FACTOR is absorbing is not
    // drag.
    public static double AIR_DENSITY = 1.225;   // kg/m^3
    public static double DRAG_COEFF = 0.47;     // sphere

    /**
     * True uses the exact quadratic-drag decay, false the linearised exponential. On, because Cd is
     * by definition a quadratic coefficient. The difference is under 1 percent at these flight
     * times, so this is correctness rather than a meaningful change.
     */
    public static boolean USE_QUADRATIC_DECAY = true;

    /**
     * Drag time constant from the ball itself, seconds.
     *
     * @param massKg       ball mass
     * @param diameterM    ball diameter, metres
     * @param speedInPerSec representative flight speed, inches/sec (exit velocity is a fair proxy)
     */
    public static double dragTauFromBall(double massKg, double diameterM, double speedInPerSec) {
        double area = Math.PI * diameterM * diameterM / 4.0;
        double v = speedInPerSec * 0.0254;
        if (area <= 0 || v <= 0) return Double.POSITIVE_INFINITY;
        return 2.0 * massKg / (AIR_DENSITY * DRAG_COEFF * area * v);
    }

    /**
     * Mass over frontal area implied by a drag time constant, kg/m^2. The inverse of the above, for
     * checking whether an empirically tuned lead factor is consistent with the real ball.
     */
    public static double massPerAreaForTau(double tau, double speedInPerSec) {
        return tau * AIR_DENSITY * DRAG_COEFF * (speedInPerSec * 0.0254) / 2.0;
    }

    /**
     * Mean surviving fraction of the inherited velocity over the flight.
     *
     * <p>Exponential form {@code (1 - e^-x)/x} for linearised drag, or {@code ln(1+x)/x} for the
     * exact quadratic solution {@code v(t) = v0/(1 + t/tau)}. The two agree within a few percent
     * while the flight is short compared with tau, and diverge past that.
     */
    private static double decayFactor(double t) {
        if (t <= 1e-6) return 1;
        double tau = Math.max(DRAG_TAU, 1e-3);
        double x = t / tau;
        return USE_QUADRATIC_DECAY ? Math.log1p(x) / x : (1 - Math.exp(-x)) / x;
    }
}
