package org.firstinspires.ftc.teamcode.Util.Controllers;

import com.acmerobotics.dashboard.config.Config;

@Config
public class velocityController {
    public static double kP = 0.004; //0.007
    public static double kPShooting = 0.004; //0.007
    public static double kI = 0;
    public  boolean shooting = true;
    public static double kD = 0;//0.00012
    public static double kV = 0.000336; ///0
    public static double kS = 0.05;
    public static double kA = 0.7; // accel feedforward
    public static boolean useKa = false;
    public static double KaErrorThreshold = 80;

    PID pid;
    public static double maxPower = 1.0;
    public static double minPower = -1.0;
    public static double maxPowerChange = 0.17;

    public static double nominalVoltage = 12.0;
    public static boolean useVoltageComp = false;
    public static boolean useBB = true;
    public static double bbPower = 1.0;
    public static double bbPowerMinus = -0.2;
    public static double bbThreeshold = 80;
    public static double bbThresholdDown = 80;

    private static double lastError = 0;
    private static double lastPower = 0;

    public static boolean pidPredict = true;
    public static double predictXThreshold = 36;
    public static double PRED_CLOSEZONE_X1 = -75, PRED_CLOSEZONE_Y1 = 83;   // top-left field corner
    public static double PRED_CLOSEZONE_X2 = -75,  PRED_CLOSEZONE_Y2 = -83;   // top-right field corner
    public static double PRED_CLOSEZONE_X3 = 7,   PRED_CLOSEZONE_Y3 = 0;    // field center

    public velocityController() {
        pid = new PID(kP,kI,kD);

    }

    public double calculate(double targetVelocity, double currentVelocity, double voltage) {
        pid.p = kP;
        if(shooting) pid.p = kPShooting;
        pid.i = kI;
        pid.d = kD;


        double feedforward = (kV * targetVelocity) + (Math.signum(targetVelocity) * kS);

        double error = targetVelocity - currentVelocity;

        if (useKa && error > KaErrorThreshold) {
            feedforward += kA * error;
        }

        if (useVoltageComp) {
            feedforward *= (nominalVoltage / voltage);
        }
        pid.setTargetValue(targetVelocity);
        double power = pid.update(currentVelocity) + feedforward;





        if(useBB) {
            if(error > bbThreeshold) {
                power = bbPower;
            }else {
                if(error < -bbThresholdDown) {
                    power = bbPowerMinus;
                }
            }
        }
        double constrainedPower = Math.max(minPower, Math.min(maxPower, power));

//        if (Math.abs(constrainedPower - lastPower) > maxPowerChange) {
//            constrainedPower = lastPower + (Math.signum(constrainedPower - lastPower) * maxPowerChange);
//        }

        lastPower = constrainedPower;
        return constrainedPower;
    }

    public static void reset() {
        lastError = 0;
        lastPower = 0;
    }

    /**
     * Returns the (x, y) point the flywheel distance/velocity lookup should use.
     * With pidPredict on and the robot on the close side (X &lt;= predictXThreshold)
     * but outside the close zone, returns the robot projected onto the nearest point
     * of the close-zone triangle, so the flywheel pre-spins to the speed it will need
     * on entry. Otherwise (predict off, far side, or already inside the zone) returns
     * the robot's actual position.
     */
    public double[] predictPoint(double px, double py) {
        if (!pidPredict || px > predictXThreshold) {
            return new double[]{px, py};
        }
        return closestPointInCloseZone(px, py);
    }

    private double[] closestPointInCloseZone(double px, double py) {
        double x1 = PRED_CLOSEZONE_X1, y1 = PRED_CLOSEZONE_Y1;
        double x2 = PRED_CLOSEZONE_X2, y2 = PRED_CLOSEZONE_Y2;
        double x3 = PRED_CLOSEZONE_X3, y3 = PRED_CLOSEZONE_Y3;

        // Already inside the close zone -> use the real position.
        if (isPointInTriangle(px, py, x1, y1, x2, y2, x3, y3)) {
            return new double[]{px, py};
        }

        double[] best = closestOnSegment(x1, y1, x2, y2, px, py);
        double bd = Math.hypot(best[0] - px, best[1] - py);

        double[] e2 = closestOnSegment(x2, y2, x3, y3, px, py);
        double d2 = Math.hypot(e2[0] - px, e2[1] - py);
        if (d2 < bd) { best = e2; bd = d2; }

        double[] e3 = closestOnSegment(x3, y3, x1, y1, px, py);
        double d3 = Math.hypot(e3[0] - px, e3[1] - py);
        if (d3 < bd) { best = e3; }

        return best;
    }

    private double[] closestOnSegment(double ax, double ay, double bx, double by, double px, double py) {
        double dx = bx - ax, dy = by - ay;
        double lsq = dx * dx + dy * dy;
        if (lsq == 0) return new double[]{ax, ay};
        double t = ((px - ax) * dx + (py - ay) * dy) / lsq;
        t = Math.max(0.0, Math.min(1.0, t));
        return new double[]{ax + t * dx, ay + t * dy};
    }

    private boolean isPointInTriangle(double x, double y, double x1, double y1, double x2, double y2, double x3, double y3) {
        double denom = (y2 - y3) * (x1 - x3) + (x3 - x2) * (y1 - y3);
        if (denom == 0) return false;

        double a = ((y2 - y3) * (x - x3) + (x3 - x2) * (y - y3)) / denom;
        double b = ((y3 - y1) * (x - x3) + (x1 - x3) * (y - y3)) / denom;
        double c = 1 - a - b;

        double eps = 1e-6;
        return a >= -eps && b >= -eps && c >= -eps;
    }
}