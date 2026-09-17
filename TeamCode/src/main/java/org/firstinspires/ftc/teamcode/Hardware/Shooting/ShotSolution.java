package org.firstinspires.ftc.teamcode.Hardware.Shooting;

/**
 * What the solver worked out for one loop. Immutable.
 *
 * <p>{@link #feasible} and {@link #requiredTurretRate} are first-class results rather than an
 * afterthought: with continuous shooting there are states where no command makes the shot, and a
 * system that fires anyway is worse than one that waits.
 */
public final class ShotSolution {

    /** Azimuth to command, field frame, radians. */
    public final double azimuthField;
    /** The same angle relative to the robot's predicted heading, which is what the turret wants. */
    public final double azimuthRelative;
    /** Rate the turret must sustain to hold the solution, rad/s. omega + v_lateral / distance. */
    public final double requiredTurretRate;
    /** Distance from the predicted shooter position to the virtual target, inches. */
    public final double virtualDistance;
    /** Straight-line distance to the real goal, inches. Range for telemetry, not for aiming. */
    public final double trueDistance;
    /** Flight time the solve converged on, seconds. */
    public final double timeOfFlight;
    /** True when the solution is inside the turret's rate and travel limits. */
    public final boolean feasible;
    /** Why it is not feasible, or "ok". */
    public final String reason;
    /** The displaced point actually aimed at, field inches. Not a physical place. */
    public final double virtualTargetX, virtualTargetY;
    /** Where the shooter is predicted to be when the ball leaves, field inches. */
    public final double shooterX, shooterY;

    public ShotSolution(double azimuthField, double azimuthRelative, double requiredTurretRate,
                        double virtualDistance, double trueDistance, double timeOfFlight,
                        boolean feasible, String reason,
                        double virtualTargetX, double virtualTargetY,
                        double shooterX, double shooterY) {
        this.virtualTargetX = virtualTargetX;
        this.virtualTargetY = virtualTargetY;
        this.shooterX = shooterX;
        this.shooterY = shooterY;
        this.azimuthField = azimuthField;
        this.azimuthRelative = azimuthRelative;
        this.requiredTurretRate = requiredTurretRate;
        this.virtualDistance = virtualDistance;
        this.trueDistance = trueDistance;
        this.timeOfFlight = timeOfFlight;
        this.feasible = feasible;
        this.reason = reason;
    }

    @Override
    public String toString() {
        return String.format("Shot[az=%.1fdeg rel=%.1fdeg rate=%.0fdeg/s d=%.1f tof=%.2f %s]",
                Math.toDegrees(azimuthField), Math.toDegrees(azimuthRelative),
                Math.toDegrees(requiredTurretRate), virtualDistance, timeOfFlight,
                feasible ? "ok" : reason);
    }
}
