package org.firstinspires.ftc.teamcode.Util.Math;

import com.arcrobotics.ftclib.util.InterpLUT;
import java.util.TreeMap;
import java.util.Map;

/**
 * 2D Interpolator for Hooded Shooters.
 * Axis 1 (Keys of TreeMap): Distance (Linear Interpolation between zones)
 * Axis 2 (Inside InterpLUT): RPM (Cubic Spline Interpolation for non-linear drop)
 */
public class MultipleRegression {

    // Key: Distance | Value: The LUT for that specific distance
    private final TreeMap<Double, InterpLUT> distanceMap = new TreeMap<>();

    // Safety bounds to prevent crashes
    private final Map<Double, Double> minRpmMap = new java.util.HashMap<>();
    private final Map<Double, Double> maxRpmMap = new java.util.HashMap<>();

    /**
     * Add a data point to the map.
     * @param distance Distance from target.
     * @param actualRpm The REAL velocity read from the encoder.
     * @param hoodAngle The servo position that worked.
     */
    public void add(double distance, double actualRpm, double hoodAngle) {
        if (!distanceMap.containsKey(distance)) {
            distanceMap.put(distance, new InterpLUT());
            minRpmMap.put(distance, Double.MAX_VALUE);
            maxRpmMap.put(distance, Double.MIN_VALUE);
        }

        // Add to the FTCLib LUT
        distanceMap.get(distance).add(actualRpm, hoodAngle);

        // Update safety bounds
        if (actualRpm < minRpmMap.get(distance)) minRpmMap.put(distance, actualRpm);
        if (actualRpm > maxRpmMap.get(distance)) maxRpmMap.put(distance, actualRpm);
    }

    /**
     * MUST be called after adding all points!
     */
    public void create() {
        for (InterpLUT lut : distanceMap.values()) {
            lut.createLUT();
        }
    }

    /**
     * Calculates the Hood Angle based on Distance and CURRENT RPM.
     */
    public double getHoodAngle(double distance, double currentRpm) {
        // 1. Find the two closest distance tables (Floor and Ceiling)
        Double lowerDist = distanceMap.floorKey(distance);
        Double upperDist = distanceMap.ceilingKey(distance);

        // Edge Case: We are closer than our shortest tuned distance
        if (lowerDist == null) return getClamped(upperDist, currentRpm);

        // Edge Case: We are farther than our farthest tuned distance
        if (upperDist == null) return getClamped(lowerDist, currentRpm);

        // Edge Case: Exact match
        if (lowerDist.equals(upperDist)) return getClamped(lowerDist, currentRpm);

        // 2. Get the angle from BOTH tables
        double angleAtLower = getClamped(lowerDist, currentRpm);
        double angleAtUpper = getClamped(upperDist, currentRpm);

        // 3. Interpolate between distances
        // (distance - lower) / (upper - lower) gives a percentage (0.0 to 1.0)
        double t = (distance - lowerDist) / (upperDist - lowerDist);

        // Lerp formula: start + percent * (end - start)
        return angleAtLower + (t * (angleAtUpper - angleAtLower));
    }

    /**
     * Helper to safely get from InterpLUT without crashing.
     * Clamps the RPM to be within the range you actually tuned.
     */
    private double getClamped(double distance, double rpm) {
        double min = minRpmMap.get(distance);
        double max = maxRpmMap.get(distance);

        // Add a tiny buffer (0.01) to ensure we are strictly inside the spline bounds
        double safeRpm = Math.max(min + 0.01, Math.min(max - 0.01, rpm));

        return distanceMap.get(distance).get(safeRpm);
    }
}