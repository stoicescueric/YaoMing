package org.firstinspires.ftc.teamcode.Util.Controllers;

import com.acmerobotics.dashboard.config.Config;

@Config
public class velocityController {
    public static double kP = 0.0035; //0.007
    public static double kPShooting = 0.007; //0.007
    public static double kI = 0;
    public  boolean shooting = true;
    public static double kD = 0;//0.00012
    public static double kV = 0.000335; ///0
    public static double kS = 0.08;

    PID pid;
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
        pid = new PID(kP,kI,kD);

    }

    public double calculate(double targetVelocity, double currentVelocity, double voltage) {
        pid.p = kP;
        if(shooting) pid.p = kPShooting;
        pid.i = kI;
        pid.d = kD;


        double feedforward = (kV * targetVelocity) + (Math.signum(targetVelocity) * kS);
        pid.setTargetValue(targetVelocity);
        double power = pid.update(currentVelocity) + feedforward;



        if (useVoltageComp) {
            power *= (nominalVoltage / voltage);
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
}