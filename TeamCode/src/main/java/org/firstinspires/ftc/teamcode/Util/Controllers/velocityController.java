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

    public static double bbThreeshold = 70;
    public velocityController() {
    }



    public static double calculate(double targetVelocity, double currentVelocity,double voltage) {
        double error = targetVelocity - currentVelocity;

        if(currentVelocity < targetVelocity - bbThreeshold) {
            return bbPower;
        }else {
            double pComp = kP * error;
            double pForward = kV * targetVelocity;

            if(useVoltageComp) return (pForward + pComp) * (12.0 / voltage);
            else return (pForward + pComp);
        }
    }


}
