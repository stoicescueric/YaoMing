package org.firstinspires.ftc.teamcode.OpMode.Tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Hardware.Outtake.OuttakePositions;
import org.firstinspires.ftc.teamcode.Util.Wrapper.TelemetryUtil;

import java.util.ArrayList;
import java.util.List;

@Disabled

@TeleOp(name = "Suge o stefanel", group = "Pedro Pathing")
@Config
public class FlywheelPidfTunerOpMode extends OpMode {

    private DcMotorEx flywheel1;
    private DcMotorEx flywheel2;
    private VoltageSensor battery;

    public static double targetTps = 1800;
    public static double emaAlpha = 0.25;
    public static double settleTimeSec = 1.2;
    public static double settleSlopeThresh = 5.0;
    public static double maxPower = 0.9;

    public static double[] ffPowerSteps = new double[]{0.25, 0.35, 0.45, 0.55, 0.65, 0.75};

    public static double kpStart = 0.001;
    public static double kpMult = 1.25;
    public static double maxKuSearchTimeSec = 60.0;
    public static double maxErrorAmplitude = 600;
    enum Phase { WAIT, FF_SWEEP, FF_DONE, PID_KU_SWEEP, PID_KU_FOUND, APPLY, DONE }
    private Phase phase = Phase.WAIT;

    private long phaseStartMs = 0;
    private double filteredVel = 0;
    private final List<Double> ffVelSamples = new ArrayList<>();
    private final List<Double> ffPowerNormSamples = new ArrayList<>();
    private double kpCurr = kpStart;
    private double kiCurr = 0.0;
    private double kdCurr = 0.0;
    private double lastError = 0.0;
    private long lastErrorTimeMs = 0;
    private final List<Long> zeroCrossTimesMs = new ArrayList<>();
    private double cycleMaxError = Double.NEGATIVE_INFINITY;
    private double cycleMinError = Double.POSITIVE_INFINITY;
    private int cyclesCounted = 0;
    private double tunedKV = OuttakePositions.kV;
    private double tunedKS = OuttakePositions.kS;
    private double Ku = 0.0;
    private double Tu = 0.0;
    private double tunedKP = OuttakePositions.kP;
    private double tunedKI = OuttakePositions.kI;
    private double tunedKD = OuttakePositions.kD;
    private double pidIntegral = 0.0;
    private double pidLastError = 0.0;
    private long pidLastTimeMs = 0;

    // Track last FF power used for safe telemetry when index has advanced
    private double lastFfPower = 0.0;

    @Override
    public void init() {
        flywheel1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        battery = hardwareMap.voltageSensor.iterator().hasNext() ? hardwareMap.voltageSensor.iterator().next() : null;
        TelemetryUtil.setup();
        flywheel2.setDirection(DcMotorEx.Direction.REVERSE);
        flywheel1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        telemetry.setMsTransmissionInterval(50);
    }

    @Override
    public void start() {
        phase = Phase.WAIT;
        phaseStartMs = System.currentTimeMillis();
        pidLastTimeMs = phaseStartMs;
        startFfSweep();
    }

    @Override
    public void loop() {
        double nowMs = System.currentTimeMillis();
        double dt = Math.max(1e-3, (nowMs - pidLastTimeMs) / 1000.0);
        pidLastTimeMs = (long) nowMs;

        double vel1 = flywheel1.getVelocity();
        double vel2 = flywheel2.getVelocity();
        double currentVel = 0.5 * (vel1 + vel2);
        filteredVel = emaAlpha * currentVel + (1 - emaAlpha) * filteredVel;

        double vBatt = battery != null ? battery.getVoltage() : 12.0;

        // Common DS telemetry
        telemetry.addData("phase", phase);
        telemetry.addData("targetTps", targetTps);
        telemetry.addData("velRaw", currentVel);
        telemetry.addData("velFilt", filteredVel);
        telemetry.addData("voltage", vBatt);

        switch (phase) {
            case WAIT:
                telemetry.addLine("A: FF sweep, X: Ku/Tu PID, B: Apply, Y: Panic stop");
                if (gamepad1.a) startFfSweep();
                if (gamepad1.x) startKuSweep();
                if (gamepad1.b) applyResults();
                break;

            case FF_SWEEP:
                runFfSweep(nowMs, vBatt);
                // Safely mirror ffStep/ffPower without indexing beyond array
                telemetry.addData("ffStep", Math.min(currentFfStepIdx, ffPowerSteps.length - 1));
                telemetry.addData("ffPower", lastFfPower);
                break;

            case FF_DONE:
                telemetry.addData("tunedKV", tunedKV);
                telemetry.addData("tunedKS", tunedKS);
                if (gamepad1.x) startKuSweep();
                if (gamepad1.b) applyResults();
                break;

            case PID_KU_SWEEP:
                runKuSweep(nowMs, dt, vBatt);
                // kpCurr/cmdPower/error are added inside runKuSweep; also mirror them there
                telemetry.addData("kpCurr", kpCurr);
                telemetry.addData("cmdPower", Math.max(0.0, Math.min(maxPower, tunedKV * targetTps + tunedKS * Math.signum(targetTps))));
                telemetry.addData("error", targetTps - filteredVel);
                break;

            case PID_KU_FOUND:
                computePidFromKuTu();
                telemetry.addData("Ku", Ku);
                telemetry.addData("Tu", Tu);
                telemetry.addData("tunedKP", tunedKP);
                telemetry.addData("tunedKI", tunedKI);
                telemetry.addData("tunedKD", tunedKD);
                break;

            case APPLY:
                telemetry.addData("APPLIED_kP", OuttakePositions.kP);
                telemetry.addData("APPLIED_kI", OuttakePositions.kI);
                telemetry.addData("APPLIED_kD", OuttakePositions.kD);
                telemetry.addData("APPLIED_kV", OuttakePositions.kV);
                telemetry.addData("APPLIED_kS", OuttakePositions.kS);
                phase = Phase.DONE;
                break;

            case DONE:
                break;
        }

        telemetry.update();
        TelemetryUtil.sendTelemetry();
        if (gamepad1.y) {
            flywheel1.setPower(0);
            flywheel2.setPower(0);
            phase = Phase.DONE;
        }
    }

    private void startFfSweep() {
        phase = Phase.FF_SWEEP;
        phaseStartMs = System.currentTimeMillis();
        ffVelSamples.clear();
        ffPowerNormSamples.clear();
        currentFfStepIdx = 0;
        lastSettleMs = 0;
        filteredVel = 0;
    }

    private int currentFfStepIdx = 0;
    private long lastSettleMs = 0;

    private void runFfSweep(double nowMs, double vBatt) {
        if (currentFfStepIdx >= ffPowerSteps.length) {
            flywheel1.setPower(0);
            flywheel2.setPower(0);
            computeFfRegression();
            phase = Phase.FF_DONE;
            return;
        }

        double p = Math.min(maxPower, ffPowerSteps[currentFfStepIdx]);
        lastFfPower = p;
        flywheel1.setPower(p);
        flywheel2.setPower(p);

        TelemetryUtil.packet.put("ffStep", currentFfStepIdx);
        TelemetryUtil.packet.put("ffPower", p);
        telemetry.addData("ffStep", currentFfStepIdx);
        telemetry.addData("ffPower", p);
        if (lastSettleMs == 0) lastSettleMs = System.currentTimeMillis();
        boolean minTime = (System.currentTimeMillis() - lastSettleMs) / 1000.0 >= settleTimeSec;
        double slope = Math.abs(filteredVel - flywheel1.getVelocity());
        boolean smallSlope = slope <= settleSlopeThresh;

        if (minTime && smallSlope) {
            double normPower = p * (12.0 / Math.max(1e-3, vBatt));
            double v = filteredVel;
            if (v > 50) {
                ffVelSamples.add(v);
                ffPowerNormSamples.add(normPower);
            }
            currentFfStepIdx++;
            lastSettleMs = System.currentTimeMillis();
        }
    }

    private void computeFfRegression() {
        int n = ffVelSamples.size();
        if (n < 2) {
            tunedKV = OuttakePositions.kV;
            tunedKS = OuttakePositions.kS;
            return;
        }
        double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
        for (int i = 0; i < n; i++) {
            double x = ffVelSamples.get(i);
            double y = ffPowerNormSamples.get(i);
            sumX += x;
            sumY += y;
            sumXY += x * y;
            sumX2 += x * x;
        }
        double denom = n * sumX2 - sumX * sumX;
        if (Math.abs(denom) < 1e-9) {
            tunedKV = OuttakePositions.kV;
            tunedKS = OuttakePositions.kS;
            return;
        }
        double slope = (n * sumXY - sumX * sumY) / denom; // kV
        double intercept = (sumY - slope * sumX) / n;     // kS
        if (slope <= 0 || intercept < 0) {
            tunedKV = OuttakePositions.kV;
            tunedKS = OuttakePositions.kS;
        } else {
            tunedKV = slope;
            tunedKS = intercept;
        }
    }

    private void startKuSweep() {
        phase = Phase.PID_KU_SWEEP;
        phaseStartMs = System.currentTimeMillis();
        kpCurr = Math.max(1e-4, kpStart);
        kiCurr = 0.0;
        kdCurr = 0.0;
        pidIntegral = 0.0;
        pidLastError = 0.0;
        zeroCrossTimesMs.clear();
        cyclesCounted = 0;
        cycleMaxError = Double.NEGATIVE_INFINITY;
        cycleMinError = Double.POSITIVE_INFINITY;
    }

    private void runKuSweep(double nowMs, double dt, double vBatt) {
        double u_ff = tunedKV * targetTps + tunedKS * Math.signum(targetTps);
        double u = u_ff;
        double error = targetTps - filteredVel;
        pidIntegral += error * dt;
        double deriv = (error - pidLastError) / dt;
        pidLastError = error;
        double u_pid = kpCurr * error + kiCurr * pidIntegral + kdCurr * deriv;
        u += u_pid;
        u = Math.max(0.0, Math.min(maxPower, u));
        flywheel1.setPower(u);
        flywheel2.setPower(u);

        TelemetryUtil.packet.put("kpCurr", kpCurr);
        TelemetryUtil.packet.put("cmdPower", u);
        TelemetryUtil.packet.put("error", error);
        telemetry.addData("kpCurr", kpCurr);
        telemetry.addData("cmdPower", u);
        telemetry.addData("error", error);
        if ((error > 0 && lastError < 0) || (error < 0 && lastError > 0)) {
            zeroCrossTimesMs.add(System.currentTimeMillis());
            double amp = Math.abs(cycleMaxError - cycleMinError) * 0.5;
            if (amp > 20) cyclesCounted++;
            cycleMaxError = Double.NEGATIVE_INFINITY;
            cycleMinError = Double.POSITIVE_INFINITY;
        }
        lastError = error;
        cycleMaxError = Math.max(cycleMaxError, error);
        cycleMinError = Math.min(cycleMinError, error);
        double ampNow = Math.abs(cycleMaxError - cycleMinError) * 0.5;
        if (ampNow > maxErrorAmplitude) {
            flywheel1.setPower(0);
            flywheel2.setPower(0);
            Ku = kpCurr / kpMult;
            Tu = estimatePeriod();
            phase = Phase.PID_KU_FOUND;
            return;
        }
        if (cyclesCounted >= 4) {
            Tu = estimatePeriod();
            if (Tu > 0) {
                Ku = kpCurr;
                phase = Phase.PID_KU_FOUND;
                return;
            }
        }
        if ((System.currentTimeMillis() - phaseStartMs) / 1000.0 > maxKuSearchTimeSec) {
            flywheel1.setPower(0);
            flywheel2.setPower(0);
            phase = Phase.FF_DONE;
            return;
        }
        if (ampNow < 10 && cyclesCounted >= 2) {
            kpCurr *= kpMult;
        }
    }

    private double estimatePeriod() {
        int n = zeroCrossTimesMs.size();
        if (n < 4) return 0.0;
        // e.g., t[i+2] - t[i]
        double sum = 0;
        int cnt = 0;
        for (int i = 0; i + 2 < n; i++) {
            long t0 = zeroCrossTimesMs.get(i);
            long t2 = zeroCrossTimesMs.get(i + 2);
            sum += (t2 - t0) / 1000.0;
            cnt++;
        }
        return cnt > 0 ? sum / cnt : 0.0;
    }

    private void computePidFromKuTu() {
        if (Ku <= 0 || Tu <= 0) {
            tunedKP = OuttakePositions.kP;
            tunedKI = OuttakePositions.kI;
            tunedKD = OuttakePositions.kD;
            return;
        }
        tunedKP = 0.3 * Ku;
        tunedKI = 0.6 * Ku / Tu;
        tunedKD = 0.0375 * Ku * Tu;
    }

    private void applyResults() {
        OuttakePositions.kV = tunedKV;
        OuttakePositions.kS = tunedKS;
        OuttakePositions.kP = tunedKP;
        OuttakePositions.kI = tunedKI;
        OuttakePositions.kD = tunedKD;
        phase = Phase.APPLY;
    }
}
