package org.firstinspires.ftc.teamcode.blob.math;

public class LowPassFilter {

    // Fixed smoothing coefficient (legacy, loop-time dependent).
    private double alpha;
    // Time constant in seconds. When > 0, the dt-aware getValue(raw, dt) uses it.
    private double tau = -1;
    private double lastValue;

    /**
     * Fixed-alpha filter. The smoothing character changes with loop time, so prefer the
     * dt-aware {@link #getValue(double, double)} for anything sensitive to loop frequency.
     */
    public LowPassFilter(double alpha, double initialValue){
        this.alpha = alpha;
        this.lastValue = initialValue;
    }

    /** dt-aware filter configured by a time constant tau (seconds). */
    public static LowPassFilter fromTimeConstant(double tau, double initialValue){
        LowPassFilter f = new LowPassFilter(0, initialValue);
        f.tau = tau;
        return f;
    }

    /** Update the time constant (seconds) live, e.g. from a dashboard variable. */
    public void setTimeConstant(double tau){
        this.tau = tau;
    }

    /** Legacy fixed-alpha update. */
    public double getValue(double rawValue){
        lastValue += alpha * (rawValue - lastValue);
        return lastValue;
    }

    /**
     * dt-aware exponential moving average.
     * alpha = dt / (tau + dt) keeps the cutoff constant regardless of loop frequency.
     */
    public double getValue(double rawValue, double dt){
        double a = (tau > 0) ? dt / (tau + dt) : alpha;
        lastValue += a * (rawValue - lastValue);
        return lastValue;
    }
}