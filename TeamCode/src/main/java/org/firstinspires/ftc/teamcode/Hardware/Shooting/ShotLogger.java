package org.firstinspires.ftc.teamcode.Hardware.Shooting;

import android.util.Log;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * One CSV row per released ball, written at the release instant.
 *
 * <p>Six coupled constants cannot be tuned by feel, which is how the old code ended up with two gain
 * knobs that cancelled. This is the record that makes the error budget computable afterwards: for
 * each shot, everything the solve depended on, so a miss can be attributed rather than guessed at.
 *
 * <p>Flushes on every row. Shots are rare compared to loops, so the cost does not matter, and a
 * power cut mid-match should not take the log with it.
 */
public class ShotLogger {

    public static final String DIR = "/sdcard/FIRST/shot-log";

    private FileWriter writer;
    private boolean failed = false;
    private int shotCount = 0;

    public ShotLogger(String label) {
        try {
            File dir = new File(DIR);
            if (!dir.exists() && !dir.mkdirs()) throw new IOException("could not create " + DIR);
            String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
            writer = new FileWriter(new File(dir, "shots_" + label + "_" + stamp + ".csv"));
            writer.write("t_ms,shot,x,y,heading,vx,vy,ax,ay,omega,"
                    + "trueDist,virtDist,tof,azField,azRel,turretRate,feasible,reason,"
                    + "targetVel,actualVel,hoodCmd,gate\n");
            writer.flush();
        } catch (Exception e) {
            failed = true;
            Log.w("ShotLogger", "disabled: " + e.getMessage());
        }
    }

    /** Records one release. Silently does nothing if the log could not be opened. */
    public void logShot(RobotState state, ShotSolution s, double targetVel, double actualVel,
                        double hoodCmd, String gateReason) {
        if (failed || writer == null) return;
        shotCount++;
        try {
            writer.write(String.format(Locale.US,
                    "%d,%d,%.2f,%.2f,%.4f,%.2f,%.2f,%.2f,%.2f,%.4f,"
                            + "%.2f,%.2f,%.4f,%.4f,%.4f,%.4f,%b,%s,"
                            + "%.1f,%.1f,%.4f,%s\n",
                    System.currentTimeMillis(), shotCount,
                    state.x, state.y, state.heading, state.vx, state.vy, state.ax, state.ay, state.omega,
                    s == null ? 0 : s.trueDistance, s == null ? 0 : s.virtualDistance,
                    s == null ? 0 : s.timeOfFlight, s == null ? 0 : s.azimuthField,
                    s == null ? 0 : s.azimuthRelative, s == null ? 0 : s.requiredTurretRate,
                    s != null && s.feasible, s == null ? "none" : s.reason,
                    targetVel, actualVel, hoodCmd, gateReason));
            writer.flush();
        } catch (IOException e) {
            failed = true;
            Log.w("ShotLogger", "write failed, disabling: " + e.getMessage());
        }
    }

    public int getShotCount() { return shotCount; }

    public void close() {
        try { if (writer != null) writer.close(); } catch (IOException ignored) { }
        writer = null;
    }
}
