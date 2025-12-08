package org.firstinspires.ftc.teamcode.Util.Wrapper;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Util.Caching.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.Util.HardwareUtils;


public class GroupMotors {
    private CachingDcMotorEx[] motors;

    public GroupMotors(CachingDcMotorEx... motors) {
        this.motors = motors;
    }

    public void setPower(double power) {
        for (CachingDcMotorEx motor : motors) {
            motor.setPower(power);
        }
    }

    public void setTargetPosition(int position) {
        for (CachingDcMotorEx motor : motors) {
            motor.setTargetPosition(position);
        }
    }
    public void resetEncoder() {
        for(CachingDcMotorEx motor : motors) {
            HardwareUtils.resetEncoder(motor);
        }
    }

    public void setMode(DcMotor.RunMode mode) {
        for (CachingDcMotorEx motor : motors) {
            motor.setMode(mode);
        }
    }

    public int getCurrentPosition() {
        return motors[0].getCurrentPosition();
    }

    public double getPower() {
        return motors[0].getPower();
    }
}
