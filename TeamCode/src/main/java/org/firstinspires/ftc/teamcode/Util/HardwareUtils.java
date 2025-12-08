package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class HardwareUtils {
    public static void unlock(DcMotorEx motor)
    {
        MotorConfigurationType mct = motor.getMotorType();
        mct.setAchieveableMaxRPMFraction(1);
        motor.setMotorType(mct);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public static void resetEncoder(DcMotorEx motor) {
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
    public static void resetMotor(DcMotorEx motor,boolean isReversed,boolean brake) {
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        unlock(motor);
        motor.setDirection(isReversed ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);
        motor.setZeroPowerBehavior(brake ? DcMotorEx.ZeroPowerBehavior.BRAKE : DcMotorEx.ZeroPowerBehavior.FLOAT);
    }
    public static void setMotorMode(DcMotorEx motor, DcMotorEx.RunMode mode) {
        motor.setMode(mode);
    }


}
