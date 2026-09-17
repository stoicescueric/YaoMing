package org.firstinspires.ftc.teamcode.Hardware.Shooting;

import com.acmerobotics.dashboard.config.Config;

/**
 * Solves the interception problem: where to point so that a ball released {@link #RELEASE_LATENCY}
 * seconds from now lands on the goal.
 *
 * <p>A pure function of state. No hardware types, no statics read from other subsystems, nothing
 * cached between calls, so the geometry can be tested on a plain JVM. This is the piece the old code
 * did not have: the same maths was spread across {@code Sensors}, {@code Turret} and {@code Launcher}
 * in two mutually exclusive copies, which is why it grew two gain knobs that cancelled each other.
 *
 * <h3>Method</h3>
 * <ol>
 *   <li>Predict the robot state forward to the release instant. Everything after this point is
 *       solved for <em>that</em> state, which is the whole content of "shooting on the move": the
 *       ball inherits the robot's velocity when it leaves, not when the aim is computed.</li>
 *   <li>Displace the goal opposite the drift the ball will inherit, and aim straight at the
 *       displaced point. In a vacuum this is exact; {@link BallisticsModel} supplies the drag
 *       correction.</li>
 *   <li>Iterate, because displacing the goal changes the distance, which changes flight time, which
 *       changes the displacement. It is a contraction, so it settles in two or three passes.</li>
 * </ol>
 *
 * <p>There is no separate stationary case. A still robot is this solve with zero velocity.
 */
@Config
public class ShotSolver {

    /**
     * Seconds between the state being true and the ball leaving: sensing lag plus command latency
     * plus servo slew plus feeder delay. The single most leveraged number here, and the one the old
     * code modelled at 0.05 while ignoring both sensing lag and slew.
     */
    public static double RELEASE_LATENCY = 0.05;

    /** Trim on the whole correction. One knob, not the old SOTM_GAIN times weightTOF pair. */
    public static double LEAD_GAIN = 1.0;

    /** Fixed-point iteration budget and its convergence threshold on flight time, seconds. */
    public static int MAX_ITERATIONS = 4;
    public static double TOF_EPSILON = 0.005;

    /**
     * Fastest azimuth rate the turret can hold, rad/s. Sets the feasibility envelope.
     *
     * <p>Axon Max MK2 at 6V is 0.115 s/60 degrees, so 522 deg/s unloaded, and the turret is driven
     * 1:1. That figure is measured with no load and no reversals, so it is derated to two thirds for
     * a turret that is actually carrying something and changing direction. Raise it once the real
     * slew rate has been measured rather than assuming the datasheet.
     */
    public static double MAX_TURRET_RATE = Math.toRadians(350);
    /**
     * Half the turret's mechanical travel, radians, from centre.
     *
     * <p>Derived from the existing servo mapping rather than picked: the command is clipped to
     * [MIN_TURRET_RANGE, MAX_TURRET_RANGE] = [0.06, 0.95] around centerPose 0.5, at
     * (MAX_TURRET_POSITION - MIN_TURRET_POSITION)/pi = 0.1798 units per radian. That works out to
     * +140 / -143 degrees, so 140 is the symmetric limit. Re-derive if any of those change.
     */
    public static double TURRET_TRAVEL_LIMIT = Math.toRadians(140);

    /** Distance below which the aim direction stops being meaningful, inches. */
    public static double MIN_DISTANCE = 1.0;

    /**
     * @param now            current field-frame state
     * @param goalX          goal position, field inches
     * @param goalY          see goalX
     * @param shooterOffset  shooter offset from robot centre along heading, inches
     * @param ballistics     flight-time and drag model
     */
    public static ShotSolution solve(RobotState now, double goalX, double goalY,
                                     double shooterOffset, BallisticsModel ballistics) {

        RobotState atRelease = now.predict(RELEASE_LATENCY);
        double sx = atRelease.shooterX(shooterOffset);
        double sy = atRelease.shooterY(shooterOffset);

        double dx = goalX - sx;
        double dy = goalY - sy;
        double trueDistance = Math.max(Math.hypot(dx, dy), MIN_DISTANCE);

        double tof = ballistics.timeOfFlight(trueDistance);
        double vtx = goalX, vty = goalY;
        double virtualDistance = trueDistance;
        double lateralVel = 0;

        for (int i = 0; i < Math.max(1, MAX_ITERATIONS); i++) {
            // Basis from the shooter to the goal. Radial error moves range, lateral error moves
            // azimuth, and the iteration exists because the two are coupled through flight time.
            double dirX = dx / trueDistance, dirY = dy / trueDistance;
            double latX = -dirY, latY = dirX;

            double radialVel = atRelease.vx * dirX + atRelease.vy * dirY;
            lateralVel = atRelease.vx * latX + atRelease.vy * latY;

            double radialDrift = radialVel * tof * ballistics.radialLeadFactor(tof) * LEAD_GAIN;
            double lateralDrift = lateralVel * tof * ballistics.lateralLeadFactor(tof) * LEAD_GAIN;

            // Move the goal against the robot's motion, which is the same as leading it.
            vtx = goalX - dirX * radialDrift - latX * lateralDrift;
            vty = goalY - dirY * radialDrift - latY * lateralDrift;

            double newDistance = Math.max(Math.hypot(vtx - sx, vty - sy), MIN_DISTANCE);
            double newTof = ballistics.timeOfFlight(newDistance);

            boolean settled = Math.abs(newTof - tof) < TOF_EPSILON;
            virtualDistance = newDistance;
            tof = newTof;
            if (settled) break;
        }

        double azimuthField = Math.atan2(vty - sy, vtx - sx);
        double azimuthRelative = wrap(azimuthField - atRelease.heading);

        // Rate the turret servo itself must slew at. azimuthRelative = azimuthField - heading, so
        // its derivative is the bearing drift from translation MINUS the robot's own spin: a robot
        // turning with the bearing needs less turret motion, not more. Unbounded as distance falls,
        // which is what makes close-range fast shooting the hard case rather than long range.
        double requiredRate = lateralVel / virtualDistance - atRelease.omega;

        boolean feasible = true;
        String reason = "ok";
        if (Math.abs(requiredRate) > MAX_TURRET_RATE) {
            feasible = false;
            reason = "turret rate";
        } else if (Math.abs(azimuthRelative) > TURRET_TRAVEL_LIMIT) {
            feasible = false;
            reason = "turret travel";
        }

        return new ShotSolution(azimuthField, azimuthRelative, requiredRate,
                virtualDistance, trueDistance, tof, feasible, reason,
                vtx, vty, sx, sy);
    }

    /** Wraps to [-pi, pi]. */
    public static double wrap(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }
}
