package org.firstinspires.ftc.teamcode.Util.Controllers;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Hardware.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.OuttakePositions;

@Config
public class Team254FlywheelController {

    public enum Mode {
        OFF,
        SPIN_UP,    // Bang-bang to get to target fast
        AT_SPEED    // Open-loop feedforward + tiny kP (the magic)
    }

    // === Feedforward (the foundation of the algorithm) ===
    public static double kV = 0.000335;       // Power per RPM — TUNE THIS FIRST
    public static double kS = 0.08;           // Static friction overcome

    // === Voltage compensation ===
    public static double nominalVoltage = 12;
    public static boolean useVoltageComp = true;

    // === Spin-up mode ===
    public static double spinUpPower = 1.0;   // Full send when below target

    // === AT_SPEED mode tuning ===
    public static double atSpeedOvershoot = 150;   // RPM above target — set to ~half your shot dip
    public static double kPHold = 0.0003;          // Gentle proportional correction (NO I, NO D)

    // === Mode transitions ===
    public static double atSpeedThreshold = 80;     // RPM error to enter AT_SPEED
    public static double targetJumpThreshold = 300; // RAISED from 100 — only big jumps drop to SPIN_UP

    public static double recoveryThreshold = 500;   // RPM error to drop back to SPIN_UP (catastrophic only)
    public static double atSpeedDebounceMs = 100;   // Must be in band this long before committing
    public static double offThreshold = 50;         // Below this target, controller is OFF

    // === Ready check ===
    public static double readyThreshold = 80;       // RPM error for "ready to shoot"

    // === Internal state ===
    private Mode mode = Mode.OFF;
    private long inBandSince = 0;
    private double lastTarget = 0;
    private double currentOutput = 0;
    public static double kPHoldMax = 0.15;
    private double lastError = 0;

    public Team254FlywheelController() {}

    /**
     * Calculate motor power for the flywheel.
     *
     * @param targetRpm   Desired flywheel velocity (ticks/sec or RPM, must match kV units)
     * @param currentRpm  Measured flywheel velocity
     * @param voltage     Current battery voltage
     * @return Motor power [0.0, 1.0]
     */
    public double calculate(double targetRpm, double currentRpm, double voltage) {
        long now = System.currentTimeMillis();
        double error = targetRpm - currentRpm;
        double absError = Math.abs(error);

        // === OFF: target too low to bother ===
        if (targetRpm < offThreshold) {
            mode = Mode.OFF;
            currentOutput = 0;
            inBandSince = 0;
            lastTarget = targetRpm;
            lastError = error;
            return 0;
        }

        // === Reset to spin-up if target jumped significantly ===
        if (Math.abs(targetRpm - lastTarget) > targetJumpThreshold && mode == Mode.AT_SPEED) {
            mode = Mode.SPIN_UP;
            inBandSince = 0;
        }
        lastTarget = targetRpm;

        // === State machine ===
        switch (mode) {
            case OFF:
                mode = Mode.SPIN_UP;
                inBandSince = 0;
                // fall through

            case SPIN_UP:
                if (error > 0) {
                    // Below target — full send
                    currentOutput = spinUpPower;
                } else {
                    // Overshooting — coast (let inertia bring us down, no braking)
                    currentOutput = 0;
                }

                // Transition check
                if (absError < atSpeedThreshold) {
                    if (inBandSince == 0) {
                        inBandSince = now;
                    } else if (now - inBandSince > atSpeedDebounceMs) {
                        mode = Mode.AT_SPEED;
                    }
                } else {
                    inBandSince = 0;
                }
                break;

            case AT_SPEED:
                // Feedforward at target + overshoot (so steady-state average lands at target)
                double effectiveTarget = targetRpm + atSpeedOvershoot;
                double ff = kV * effectiveTarget + Math.signum(effectiveTarget) * kS;

                // Voltage compensation on feedforward only
                if (useVoltageComp && voltage > 1.0) {
                    ff *= (nominalVoltage / voltage);
                }

                // Gentle proportional correction toward actual target (no I, no D)
                double p = kPHold * error;
                p = Math.max(-kPHoldMax, Math.min(kPHoldMax, p));


                currentOutput = ff + p;

                // Drop back to spin-up only on catastrophic events
                if (absError > recoveryThreshold) {
                    mode = Mode.SPIN_UP;
                    inBandSince = 0;
                }
                break;
        }

        // Clamp
        currentOutput = Math.max(0.0, Math.min(1.0, currentOutput));
        lastError = error;

        return currentOutput;
    }

    /** True when controller is in steady-state hold mode and within ready threshold. */
    public boolean isReady(double targetRpm, double currentRpm) {
        return mode == Mode.AT_SPEED && Math.abs(targetRpm - currentRpm) < OuttakePositions.errorVelThreeshold;
    }

    /** True when controller is in AT_SPEED mode (regardless of momentary error). */
    public boolean isAtSpeed() {
        return mode == Mode.AT_SPEED;
    }

    public Mode getMode() {
        return mode;
    }

    public double getLastError() {
        return lastError;
    }

    public double getCurrentOutput() {
        return currentOutput;
    }

    public void reset() {
        mode = Mode.OFF;
        inBandSince = 0;
        currentOutput = 0;
        lastTarget = 0;
        lastError = 0;
    }
}