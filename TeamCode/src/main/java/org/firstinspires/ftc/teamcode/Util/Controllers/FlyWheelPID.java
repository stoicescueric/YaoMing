package org.firstinspires.ftc.teamcode.Util.Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
// Use Panels config holder
import org.firstinspires.ftc.teamcode.Hardware.Intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.OuttakePositions;




/**
 * Reusable dual-motor flywheel PID controller with feedforward.
 *
 * Usage example:
 *   // In your opmode or subsystem:
 *   DcMotorEx motor1 = hardwareMap.get(DcMotorEx.class, "launcher1");
 *   DcMotorEx motor2 = hardwareMap.get(DcMotorEx.class, "launcher2");
 *
 *   FlywheelPID fly = new FlywheelPID(motor1, motor2);
 *   fly.setGains(0.00278, 0.0, 0.0, 0.0, 0.00044, 0.0); // Kp, Ki, Kd, Ks, Kv, Ka
 *
 *   double targetTicksPerSec = 1633;
 *
 *   // In your control loop:
 *   while (opModeIsActive()) {
 *       fly.update(targetTicksPerSec);
 *   }
 */
@Config
public class FlyWheelPID {


    // PID gains (feedback control)
    public double Kp = 0.00278;  // proportional gain
    public double Ki = 0.0;      // integral gain
    public double normalize_voltage = 12.0;
    public double Kd = 0.0;      // derivative gain

    // Feedforward gains
    public double Ks = 0.0;      // static friction compensation
    public double Kv = 0.00044;  // velocity feedforward
    public double Ka = 0.0;      // acceleration feedforward

    // Power limits
    public double maxPower = 1.0;
    public double minPower = -1.0;

    // Internal state
    private double previousError = 0.0;
    private double integralSum = 0.0;
    private double previousVelocity = 0.0;
    private final ElapsedTime pidTimer = new ElapsedTime();

    // Encoder CPR (ticks per revolution) - adjust to your motor's encoder
    // Common values: goBILDA 5202/5203/5204 = 537.6, NeveRest = 1120/560/1680, REV Through Bore = 8192
    public static double TICKS_PER_REV = 537.6; // default goBILDA, tune to your motor


    public FlyWheelPID() {

        pidTimer.reset();
    }

    /**
     * Reset PID state (integrator, derivative tracking, timer).
     * Call this when starting/stopping the flywheel or when changing targets significantly.
     */
    public void reset() {
        previousError = 0.0;
        integralSum = 0.0;
        previousVelocity = 0.0;
        pidTimer.reset();
    }

    /**
     * Update motors to achieve target velocity (ticks per second) using motor1's encoder.
     * Call this periodically in your control loop (e.g., each iteration).
     *
     * @param targetTicksPerSec Desired flywheel velocity in encoder ticks per second.
     */
    public double update(double targetTicksPerSec,double velocity,double current_voltage) {

        // Pull latest config each cycle
        double KpCfg = OuttakePositions.kP;
        double KiCfg = OuttakePositions.kI;
        double KdCfg = OuttakePositions.kD;
        double KsCfg = OuttakePositions.kS;
        double KvCfg = OuttakePositions.kV;
        double KaCfg = 0;
        double minPowerCfg = -1;
        double maxPowerCfg = 1;

        double currentVelocity = velocity;
        double error = targetTicksPerSec - currentVelocity;

        double dt = pidTimer.seconds();
        pidTimer.reset();
        if (dt <= 0 || dt > 0.1) dt = 0.02; // default to ~20ms if invalid

        // PID using config values
        double proportional = KpCfg * error;

        integralSum += error * dt;
        double integral = KiCfg * integralSum;

        double derivative = (dt > 0) ? KdCfg * (error - previousError) / dt : 0.0;

        // Feedforward using config values
        double acceleration = (targetTicksPerSec - previousVelocity) / dt;
        double feedforward = KsCfg * Math.signum(targetTicksPerSec) +
                KvCfg * targetTicksPerSec +
                KaCfg * acceleration;

        // Compute total output and clamp
        double output = proportional + integral + derivative + feedforward;
        output= output * (normalize_voltage/current_voltage);
        output = clamp(output, minPowerCfg, maxPowerCfg);

        // Apply power to both motors


        // Update state for next iteration
        previousError = error;
        previousVelocity = currentVelocity;

        // Anti-windup with config limits
        if (Math.abs(output) >= maxPowerCfg || Math.abs(output) <= minPowerCfg) {
            integralSum = 0.0;
        }
        return output;
    }

    /**
     * Set PID and feedforward gains.
     *
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     * @param ks Static friction compensation (feedforward)
     * @param kv Velocity feedforward gain
     * @param ka Acceleration feedforward gain
     */
    public void setGains(double kp, double ki, double kd, double ks, double kv, double ka) {
        this.Kp = kp;
        this.Ki = ki;
        this.Kd = kd;
        this.Ks = ks;
        this.Kv = kv;
        this.Ka = ka;
    }

    /**
     * Set motor power limits.
     *
     * @param min Minimum power (typically -1.0)
     * @param max Maximum power (typically 1.0)
     */
    public void setPowerLimits(double min, double max) {
        this.minPower = min;
        this.maxPower = max;
    }

    /**
     * Get the current velocity from motor1's encoder.
     *
     * @return Current velocity in ticks per second, or 0.0 if unavailable.
     */

    /**
     * Get the current RPM of the flywheel (1:1 bare motor).
     *
     * @return Current RPM based on motor1's encoder velocity.
     */

    /**
     * Convert ticks per second to RPM (1:1 ratio, no gearbox).
     *
     * @param ticksPerSecond Encoder velocity in ticks/sec
     * @return RPM
     */
    public static double ticksPerSecondToRPM(double ticksPerSecond) {
        return (ticksPerSecond / TICKS_PER_REV) * 60.0;
    }

    /**
     * Convert RPM to ticks per second (1:1 ratio, no gearbox).
     *
     * @param rpm Target RPM
     * @return Ticks per second
     */
    public static double rpmToTicksPerSecond(double rpm) {
        return (rpm * TICKS_PER_REV) / 60.0;
    }

    /**
     * Stop both motors by setting power to 0.
     */


    // Helper methods

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private static double safeGetVelocity(DcMotorEx motor) {
        try {
            return motor.getVelocity();
        } catch (Exception e) {
            return 0.0;
        }
    }
}