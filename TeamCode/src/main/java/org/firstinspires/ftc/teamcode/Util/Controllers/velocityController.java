package org.firstinspires.ftc.teamcode.Util.Controllers;

import com.acmerobotics.dashboard.config.Config;

@Config
public class velocityController {
    public static double kP = 0.0014;
    public static double kI = 0;
    public static double kD = 0.0001;
    public static double kV = 0.00035;
    public static double kS = 0.06;

    public static double maxPower = 1.0;
    public static double minPower = -1.0;
    public static double maxPowerChange = 0.17;

    public static double nominalVoltage = 12.0;
    public static boolean useVoltageComp = false;
    public static boolean useBB = false;
    public static double bbPower = 1.0;
    public static double bbThreeshold = 100;

    private static double lastError = 0;
    private static double lastPower = 0;

    public velocityController() {
    }

    public static double calculate(double targetVelocity, double currentVelocity, double voltage) {
        double error = targetVelocity - currentVelocity;

        double derivative = error - lastError;
        lastError = error;

        double feedforward = (kV * targetVelocity) + (Math.signum(targetVelocity) * kS);
        double feedback = (kP * error) + (kD * derivative);



        if (useVoltageComp) {
            feedforward *= (nominalVoltage / voltage);
        }
        double targetPower = feedforward + feedback;
        if (useBB && (currentVelocity < targetVelocity - bbThreeshold)) {
            targetPower = bbPower;
        }

        double constrainedPower = Math.max(minPower, Math.min(maxPower, targetPower));

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
}