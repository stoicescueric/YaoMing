package org.firstinspires.ftc.teamcode.Hardware.Shooting;

/**
 * The robot's motion state at one instant, in field coordinates.
 *
 * <p>Immutable and free of hardware types so the shot maths can be exercised on a plain JVM. Heading
 * is the raw field heading (CCW positive), not blob's mirrored convention: blob mirrors internally
 * for its own heading error, and {@code Sensors} reads the unmirrored value, so field coordinates
 * and field headings stay consistent everywhere in this package.
 */
public final class RobotState {

    public final double x, y, heading;
    public final double vx, vy;
    public final double ax, ay;
    public final double omega;

    public RobotState(double x, double y, double heading,
                      double vx, double vy,
                      double ax, double ay,
                      double omega) {
        this.x = x; this.y = y; this.heading = heading;
        this.vx = vx; this.vy = vy;
        this.ax = ax; this.ay = ay;
        this.omega = omega;
    }

    /**
     * Constant-acceleration extrapolation by {@code dt} seconds.
     *
     * <p>This is how the release latency is applied: solve for the state the robot will be in when
     * the ball actually leaves, not the state it is in when the aim is computed. Acceleration is
     * held constant over the step, which is fine because the step is tens of milliseconds.
     */
    public RobotState predict(double dt) {
        if (dt == 0) return this;
        return new RobotState(
                x + vx * dt + 0.5 * ax * dt * dt,
                y + vy * dt + 0.5 * ay * dt * dt,
                heading + omega * dt,
                vx + ax * dt,
                vy + ay * dt,
                ax, ay,
                omega);
    }

    /** Shooter position, offset from the robot centre along its heading. */
    public double shooterX(double forwardOffset) { return x + forwardOffset * Math.cos(heading); }

    /** @see #shooterX(double) */
    public double shooterY(double forwardOffset) { return y + forwardOffset * Math.sin(heading); }

    public double speed() { return Math.hypot(vx, vy); }

    @Override
    public String toString() {
        return String.format("RobotState[x=%.1f y=%.1f h=%.2f v=(%.1f,%.1f) a=(%.1f,%.1f) w=%.2f]",
                x, y, heading, vx, vy, ax, ay, omega);
    }
}
