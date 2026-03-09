package org.firstinspires.ftc.teamcode.Util.Controllers;

import com.acmerobotics.dashboard.config.Config;

@Config
public class velocityController {
    public static double kP = 0.006;
    public static double kI = 0;
    public static double kD = 0;
    public static double kV = 0.00035;
    public static double kS = 0.06;

    public static double bbPower = 1.0;
    public static double bbPowerMin = -0.4;

    public static boolean useVoltageComp = true;

    public static double maxPower = 1.0;

    public static double minPower = -1.0;

    public static double bbThreeshold = 100;
    public static boolean useBB = false;
    public velocityController() {
    }



    public static double calculate(double targetVelocity, double currentVelocity,double voltage) {
        double error = targetVelocity - currentVelocity;
        double pidPower = kP * error + kV * targetVelocity;
        if(useVoltageComp) {
            pidPower *= (12.0 / voltage);
        }

        if(!useBB) {
            return pidPower;
        }
        if(currentVelocity < targetVelocity - bbThreeshold) {
            return bbPower;
        }else {
            double pComp = kP * error;
            double pForward = kV * targetVelocity;
            return pidPower;
        }
    }


}
