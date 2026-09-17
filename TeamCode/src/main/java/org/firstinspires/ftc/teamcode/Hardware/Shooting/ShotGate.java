package org.firstinspires.ftc.teamcode.Hardware.Shooting;

import com.acmerobotics.dashboard.config.Config;

/**
 * Decides whether the robot is allowed to release a ball right now.
 *
 * <p>Separate from the solver on purpose. The solver answers "where do I point"; the gate answers
 * "is pointing there going to score", and with continuous shooting that second question needs an
 * owner. The old code only asked whether the flywheel was within a fixed velocity band.
 *
 * <h3>Tolerances scale with distance</h3>
 * One degree of azimuth error is 0.9 inches of miss at 50 inches and 2.6 at 150. A fixed angular
 * tolerance is therefore three times looser at range, which is exactly backwards. The gate works in
 * inches of predicted miss instead, so one number means the same thing everywhere on the field.
 */
@Config
public class ShotGate {

    /** Allowed lateral miss from azimuth error, inches at the goal. */
    public static double AZIMUTH_MISS_TOLERANCE_IN = 2.0;
    /** Allowed flywheel error, ticks/sec. */
    public static double VELOCITY_TOLERANCE_TPS = 35;
    /** Allowed hood error, servo units. */
    public static double HOOD_TOLERANCE = 0.02;
    /** Requires the solution to stay feasible rather than only be feasible this instant. */
    public static boolean REQUIRE_FEASIBLE = false;
    /** Set false to fall back to the old flywheel-only check, for A/B on the robot. */
    public static boolean ENFORCE_AIM = false;

    public String lastReason = "not evaluated";

    /**
     * @param solution     this loop's solve
     * @param azimuthError commanded minus actual turret azimuth, radians
     * @param velocityErr  commanded minus actual flywheel velocity, ticks/sec
     * @param hoodError    commanded minus actual hood, servo units
     */
    public boolean allowRelease(ShotSolution solution, double azimuthError,
                                double velocityErr, double hoodError) {
        if (solution == null) {
            lastReason = "no solution";
            return false;
        }
        if (REQUIRE_FEASIBLE && !solution.feasible) {
            lastReason = solution.reason;
            return false;
        }
        if (Math.abs(velocityErr) > VELOCITY_TOLERANCE_TPS) {
            lastReason = "flywheel";
            return false;
        }
        if (ENFORCE_AIM) {
            // Small-angle: miss at the goal is roughly the angular error times the range.
            double azimuthMiss = Math.abs(azimuthError) * solution.virtualDistance;
            if (azimuthMiss > AZIMUTH_MISS_TOLERANCE_IN) {
                lastReason = "azimuth";
                return false;
            }
            if (Math.abs(hoodError) > HOOD_TOLERANCE) {
                lastReason = "hood";
                return false;
            }
        }
        lastReason = "ok";
        return true;
    }
}
