package org.firstinspires.ftc.teamcode.Watchdog;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.watchdog.Watchdog;

/**
 * Lightweight pose logger that uses Watchdog to record robot (x,y,heading) each loop.
 */
@Config
public final class PoseLogger {
    private PoseLogger() {}

    public static String CHANNEL = "pose";

    private static String currentRunId = null;
    private static long startTimestamp = 0L;

    public static void startRun(String label) {
        startTimestamp = System.currentTimeMillis();
        currentRunId = label + "_" + startTimestamp;
        Watchdog.log(CHANNEL,
                "event=startRun,runId=" + currentRunId + ",label=" + label,
                "meta");
    }

    public static void endRun() {
        if (currentRunId != null) {
            Watchdog.log(CHANNEL,
                    "event=endRun,runId=" + currentRunId,
                    "meta");
        }
    }

    /**
     * Log the given pose with a timestamp. Payload format:
     *   runId,timestampMillis,x,y,heading
     */
    public static void logPose(Pose pose) {
        if (pose == null || currentRunId == null) return;
        long t = System.currentTimeMillis();
        String payload = currentRunId + "," + t + "," + pose.getX() + "," + pose.getY() + "," + pose.getHeading();
        Watchdog.log(CHANNEL, payload, "pose");
    }

    public static String getCurrentRunId() {
        return currentRunId;
    }
}
