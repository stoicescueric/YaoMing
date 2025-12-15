package org.firstinspires.ftc.teamcode.Util.Math;

import  java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Performs spline interpolation given a set of control points, with optional
 * linear extrapolation beyond the min/max X-values.
 */
public class Regression {

    private List<Double> mX = new ArrayList<>();
    private List<Double> mY = new ArrayList<>();
    private List<Double> mM = new ArrayList<>();

    public Regression() {
        // Default constructor
    }

    /**
     * Private constructor if you ever want to create from existing lists.
     */
    private Regression(List<Double> x, List<Double> y, List<Double> m) {
        mX = x;
        mY = y;
        mM = m;
    }

    /**
     * Add a single (x, y) data point to the LUT.
     *
     * @param input  The x-value (e.g., arm extension)
     * @param output The y-value (e.g., feedforward needed at that extension)
     */
    public void add(double input, double output) {
        mX.add(input);
        mY.add(output);
    }

    /**
     * Call this AFTER adding all your (x, y) data points to prepare the
     * monotonic cubic spline coefficients (mM).
     *
     * <p>Requires that your x-values are strictly increasing. Otherwise,
     * it will throw an IllegalArgumentException.</p>
     */
    public void createLUT() {
        if (mX == null || mY == null || mX.size() != mY.size() || mX.size() < 2) {
            throw new IllegalArgumentException(
                    "There must be at least two control points and the arrays must be of equal length."
            );
        }

        final int n = mX.size();

        // Sort the points by x if not already sorted (Optional—depends on your usage).
        // If you are sure your data is already in ascending order, you can skip.
        // But typically you'd want to ensure sorted data for the spline:
        // sortDataByX(); // Implement if needed.

        // Compute secant slopes of successive points
        Double[] d = new Double[n - 1];
        for (int i = 0; i < n - 1; i++) {
            double h = mX.get(i + 1) - mX.get(i);
            if (h <= 0) {
                throw new IllegalArgumentException(
                        "The control points must have strictly increasing X values."
                );
            }
            d[i] = (mY.get(i + 1) - mY.get(i)) / h;
        }

        // Initialize the tangents as the average of the secants
        Double[] m = new Double[n];
        m[0] = d[0];
        for (int i = 1; i < n - 1; i++) {
            m[i] = 0.5 * (d[i - 1] + d[i]);
        }
        m[n - 1] = d[n - 2];

        // Update tangents to preserve monotonicity
        for (int i = 0; i < n - 1; i++) {
            if (d[i] == 0f) {
                // Successive Y values are equal
                m[i] = 0.0;
                m[i + 1] = 0.0;
            } else {
                double a = m[i] / d[i];
                double b = m[i + 1] / d[i];
                // If the vector (a, b) is too long, rescale it to length 3
                double h = Math.hypot(a, b);
                if (h > 9.0) {
                    double t = 3.0 / h;
                    m[i] = t * a * d[i];
                    m[i + 1] = t * b * d[i];
                }
            }
        }

        // Store final tangents
        mM = Arrays.asList(m);
    }

    /**
     * Interpolates (or extrapolates) the value of Y = f(X).
     * <p>
     * Within the bounds [x0, xN], this does the usual monotonic cubic spline.
     * Outside those bounds, it linearly extrapolates using the first or last two points.
     *
     * @param input the X value for which you want an interpolated (or extrapolated) Y.
     * @return the spline (or linear-extrapolated) value at 'input'.
     */
    public double get(double input) {
        final int n = mX.size();
        if (Double.isNaN(input)) {
            return input; // or handle NaN differently
        }

        // 1. Below the minimum X => linear extrapolation from first two points
        if (input < mX.get(0)) {
            return linearExtrapolate(
                    input,
                    mX.get(0), mY.get(0),
                    mX.get(1), mY.get(1)
            );
        }

        // 2. Above the maximum X => linear extrapolation from last two points
        if (input > mX.get(n - 1)) {
            return linearExtrapolate(
                    input,
                    mX.get(n - 2), mY.get(n - 2),
                    mX.get(n - 1), mY.get(n - 1)
            );
        }

        // If exactly at the first or last point, just return that y
        if (input == mX.get(0)) {
            return mY.get(0);
        }
        if (input == mX.get(n - 1)) {
            return mY.get(n - 1);
        }

        // Find the segment [x_i, x_{i+1}] containing 'input'
        int i = 0;
        while (input >= mX.get(i + 1)) {
            i++;
            // If it's exactly at x_i, just return y_i
            if (input == mX.get(i)) {
                return mY.get(i);
            }
        }

        // Perform cubic Hermite interpolation
        double h = mX.get(i + 1) - mX.get(i);
        double t = (input - mX.get(i)) / h;

        return (mY.get(i) * (1 + 2 * t) + h * mM.get(i) * t) * Math.pow((1 - t), 2)
                + (mY.get(i + 1) * (3 - 2 * t) + h * mM.get(i + 1) * (t - 1)) * t * t;
    }

    /**
     * Helper function for linear interpolation/extrapolation between two known points.
     */
    private double linearExtrapolate(double queryX,
                                     double x1, double y1,
                                     double x2, double y2) {
        double slope = (y2 - y1) / (x2 - x1);
        return y1 + slope * (queryX - x1);
    }

    /**
     * (Optional) Public getters if you need direct access to the internal arrays
     * for debugging or clamping in external code.
     */
    public List<Double> getX() {
        return mX;
    }

    public List<Double> getY() {
        return mY;
    }

    /**
     * For debugging: prints out the (x, y) pairs plus tangents.
     */
    @Override
    public String toString() {
        StringBuilder str = new StringBuilder();
        final int n = mX.size();
        str.append("[");
        for (int i = 0; i < n; i++) {
            if (i != 0) {
                str.append(", ");
            }
            str.append("(").append(mX.get(i));
            str.append(", ").append(mY.get(i));
            if (i < mM.size()) {
                str.append(": ").append(mM.get(i));
            }
            str.append(")");
        }
        str.append("]");
        return str.toString();
    }
}