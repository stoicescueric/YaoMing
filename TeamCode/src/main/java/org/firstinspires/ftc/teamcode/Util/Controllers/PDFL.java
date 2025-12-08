package org.firstinspires.ftc.teamcode.Util.Controllers;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PDFL {

    private double kP, kD, kF, kL;
    private double deadzone;
    private double homedConstant;
    private boolean homed = false;
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private RingBuffer<Double> timeBuffer = new RingBuffer<Double>(3, 0.0);
    private RingBuffer<Double> errorBuffer = new RingBuffer<Double>(3, 0.0);
    private double filteredDerivative = 0.0;
    private double alpha = 0.1; // Smoothing factor for the low pass filter

    public PDFL(double kP, double kD, double kF, double kL) {
        this.kP = kP;
        this.kD = kD;
        this.kF = kF;
        this.kL = kL;
    }

    public void updateConstants(double kP, double kD, double kF, double kL) {
        this.kP = kP;
        this.kD = kD;
        this.kF = kF;
        this.kL = kL;
    }

    public void setDeadzone(double deadzone) {
        this.deadzone = deadzone;
    }

    public void setHomed(boolean homed) {
        this.homed = homed;
    }

    public void setHomedConstant(double constant) {
        homedConstant = constant;
    }

    public void reset() {
        timeBuffer.fill(0.0);
        errorBuffer.fill(0.0);
        timer.reset();
        filteredDerivative = 0.0;
    }

    public double run(double error) {
        if (homed) {
            return homedConstant;
        }

        double time = timer.time();
        double previous_time = timeBuffer.getValue(time);
        double previous_error = errorBuffer.getValue(error);
        double delta_time = time - previous_time;
        double delta_error = error - previous_error;

        // If the PDFL hasn't been updated, reset it
        if (delta_time > 200) {
            reset();
            return run(error);
        }

        double p = pComponent(error);
        double d = dComponent(delta_error, delta_time);
        double f = fComponenet();
        double l = lComponent(error);

        double response = p + d + f + l;

        if (Math.abs(error) < deadzone) {
            // Same response but without lower limit
            response = p + d + f;
        }

        return response;
    }

    private double pComponent(double error) {
        return kP * error;
    }

    private double dComponent(double delta_error, double delta_time) {
        double derivative = delta_error / delta_time;
        filteredDerivative = alpha * derivative + (1 - alpha) * filteredDerivative;
        return filteredDerivative * kD;
    }

    private double fComponenet() {
        return kF;
    }

    private double lComponent(double error) {
        double direction = Math.signum(error);
        return direction * kL;
    }
}