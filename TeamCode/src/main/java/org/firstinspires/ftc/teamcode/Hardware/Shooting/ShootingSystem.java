package org.firstinspires.ftc.teamcode.Hardware.Shooting;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.Sensors;

/**
 * Owns the shot solution for the loop and hands it to whoever needs it.
 *
 * <p>One place assembles the state, runs the solve, runs the gate and writes the log. The turret and
 * the launcher read the answer instead of each recomputing their own slice of it, which is what the
 * old code did across three classes and two duplicate code paths.
 *
 * <p>{@link #USE_SOLVER} is false by default. With it off this class still solves and logs every
 * loop, but nothing consumes the answer, so the robot behaves exactly as before and the two paths
 * can be compared side by side on telemetry before the cutover.
 */
@Config
public class ShootingSystem {

    /** Turret and launcher follow the solver when true, the legacy path when false. */
    public static boolean USE_SOLVER = false;
    /** Runs the solve even while USE_SOLVER is off, so telemetry can compare the two. */
    public static boolean ALWAYS_SOLVE = true;
    /** Writes a CSV row per released ball. */
    public static boolean LOG_SHOTS = true;

    private final Robot robot;
    private final Sensors sensors;

    public final BallisticsModel ballistics = new BallisticsModel();
    public final VelocityObserver observer = new VelocityObserver();
    public final ShotGate gate = new ShotGate();

    private ShotLogger logger;
    private ShotSolution solution;
    private RobotState state;

    public ShootingSystem(Robot robot, Sensors sensors) {
        this.robot = robot;
        this.sensors = sensors;
    }

    /** Opens the shot log. Call once from an OpMode's start, so init does not create empty files. */
    public void startLogging(String label) {
        if (!LOG_SHOTS || logger != null) return;
        logger = new ShotLogger(label);
    }

    public void stopLogging() {
        if (logger != null) logger.close();
        logger = null;
    }

    /** Assembles the state and solves. Called once per loop, after Sensors has updated. */
    public void update() {
        observer.update(
                robot.blob.getWheelCommand(),
                robot.blob.getBatteryVoltage(),
                sensors.getHeading(),
                robot.blob.getVelocityX(),
                robot.blob.getVelocityY(),
                robot.blob.odo.getAccX(),
                robot.blob.odo.getAccY());

        state = new RobotState(
                sensors.getX(), sensors.getY(), sensors.getHeading(),
                robot.blob.getVelocityX(), robot.blob.getVelocityY(),
                observer.fusedAx, observer.fusedAy,
                sensors.getAngularVelocity());

        if (USE_SOLVER || ALWAYS_SOLVE) {
            // The RAW goal, not getTargetX(). That getter returns the legacy path's virtual target
            // whenever sotm is on, and feeding an already-corrected point into the solver would
            // apply the lead twice. In TeleOp sotm is the left trigger, so this is one pull away.
            solution = ShotSolver.solve(state, sensors.targetX, sensors.targetY,
                    Sensors.FORWARD_TURRET_OFFSET, ballistics);
        } else {
            solution = null;
        }
    }

    /** This loop's solution, or null if solving is switched off entirely. */
    public ShotSolution getSolution() { return solution; }

    /** This loop's state, as handed to the solver. */
    public RobotState getState() { return state; }

    /** True when the turret and launcher should be following the solver. */
    public boolean solverActive() { return USE_SOLVER && solution != null; }

    /**
     * Whether a ball may be released now. Falls back to the flywheel-only test when there is no
     * solution, so a solver failure cannot wedge the feed shut.
     */
    public boolean allowRelease(double azimuthError, double velocityError, double hoodError) {
        if (solution == null) return Math.abs(velocityError) <= ShotGate.VELOCITY_TOLERANCE_TPS;
        return gate.allowRelease(solution, azimuthError, velocityError, hoodError);
    }

    /** Records one released ball. */
    public void logShot(double targetVel, double actualVel, double hoodCmd) {
        if (logger != null && state != null) {
            logger.logShot(state, solution, targetVel, actualVel, hoodCmd, gate.lastReason);
        }
    }

    public int getShotCount() { return logger == null ? 0 : logger.getShotCount(); }

    // Field overlay colours. Amber is the solver, pink is whatever the legacy path is aiming at.
    private static final String C_SOLVER   = "#ffcc00";
    private static final String C_LEGACY   = "#ff3366";
    private static final String C_DISAGREE = "#ffffff";
    private static final String C_SHOOTER  = "#00ff88";

    /**
     * Draws the two aim points side by side so they can be compared while driving.
     *
     * <p>The white line between them is the whole point: with the robot still it should collapse to
     * nothing, and while moving its length is how far apart the two models think the shot is. The
     * green line is the shooter's predicted travel over the release latency, which is the part of
     * the correction the legacy path folds into its drift term instead of predicting.
     *
     * @param field       the dashboard field overlay
     * @param legacyAimX  what the legacy path aims at, i.e. sensors.getTargetX()
     * @param legacyAimY  see legacyAimX
     * @param shooterNowX shooter position right now, before the latency prediction
     * @param shooterNowY see shooterNowX
     */
    public void drawField(Canvas field, double legacyAimX, double legacyAimY,
                          double shooterNowX, double shooterNowY) {
        if (field == null || solution == null) return;

        // Where the solver says to aim.
        field.setStroke(C_SOLVER);
        field.strokeCircle(solution.virtualTargetX, solution.virtualTargetY, 3);
        field.strokeLine(solution.shooterX, solution.shooterY,
                solution.virtualTargetX, solution.virtualTargetY);

        // Where the legacy path says to aim, drawn a touch larger so the two never hide each other.
        field.setStroke(C_LEGACY);
        field.strokeCircle(legacyAimX, legacyAimY, 4);

        // The disagreement itself.
        field.setStroke(C_DISAGREE);
        field.strokeLine(legacyAimX, legacyAimY, solution.virtualTargetX, solution.virtualTargetY);

        // Shooter now, and where it is predicted to be when the ball actually leaves.
        field.setStroke(C_SHOOTER);
        field.strokeCircle(solution.shooterX, solution.shooterY, 1.5);
        field.strokeLine(shooterNowX, shooterNowY, solution.shooterX, solution.shooterY);
    }

    /** Inches between the two aim points. Zero when stationary; grows with speed. */
    public double aimDisagreement(double legacyAimX, double legacyAimY) {
        if (solution == null) return 0;
        return Math.hypot(solution.virtualTargetX - legacyAimX, solution.virtualTargetY - legacyAimY);
    }
}
