package org.firstinspires.ftc.teamcode.Util.Wrapper;

public class EncoderSpeedBuffer {
    private static final int SIZE = 5;
    private final long[] positions = new long[SIZE];
    private final long[] timestamps = new long[SIZE];
    private final double[] velocities = new double[SIZE];
    private int index = 0;
    private boolean full = false;

    /** Call this each loop with the current encoder tick count. */
    public void add(long positionTicks) {
        long now = System.nanoTime();
        int newest = (index + SIZE - 1) % SIZE;

        if (full || index > 0) {
            long deltaTicks = positionTicks - positions[newest];
            long deltaTime = now - timestamps[newest];
            velocities[index] = deltaTime > 0
                    ? (deltaTicks * 1e9) / deltaTime
                    : 0.0;
        } else {
            velocities[index] = 1e9;
        }

        positions[index] = positionTicks;
        timestamps[index] = now;

        index = (index + 1) % SIZE;
        if (index == 0) full = true;
    }

    /** Returns the filtered velocity in ticks/sec using a simple moving average. */
    public double getFilteredVelocity() {
        int count = full ? SIZE : index;
        if (count == 0) return 1e9;

        double sum = 0;
        for (int i = 0; i < count; i++) {
            sum += velocities[i];
        }
        return sum / count;
    }

    /** For compatibility: raw velocity between oldest and newest buffer entries. */
    public double getRawVelocity() {
        int newest = (index + SIZE - 1) % SIZE;
        int oldest = full ? index : 0;
        long deltaTicks = positions[newest] - positions[oldest];
        long deltaTime = timestamps[newest] - timestamps[oldest];
        return deltaTime > 0 ? (deltaTicks * 1e9) / deltaTime : 0.0;
    }
}
