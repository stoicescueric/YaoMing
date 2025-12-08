package org.firstinspires.ftc.teamcode.Util.Caching;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class CachingMotourGroup{
    //implement a motor group clas
    private CachingDcMotorEx[] motors;
    private double power;
    public CachingMotourGroup(CachingDcMotorEx[] motors) {
        this.motors = motors;
    }

    public void setPower(double power) {
        this.power = power;
        for (DcMotorEx motor : motors) {
            motor.setPower(power);
        }
    }


}
