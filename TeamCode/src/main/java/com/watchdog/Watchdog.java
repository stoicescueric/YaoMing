package com.watchdog;

import android.content.Context;
import android.content.Intent;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

/**
 * Public entry point for the Watchdog library. Teams can edit this file to tweak configuration.
 */
public final class Watchdog {
    private Watchdog() {}

    // === Editable configuration ===
    public static boolean AUTO_START = true;
    public static boolean ENABLE_HTTP_SERVER = true;
    public static int HTTP_PORT = 8024;
    public static int QUEUE_CAPACITY = 1024;
    public static boolean DROP_WHEN_QUEUE_FULL = true;
    public static int MAX_BATCH_SIZE = 32;
    public static long BATCH_FLUSH_INTERVAL_MS = 25;
    public static long PAUSE_POLL_INTERVAL_MS = 15;
    public static boolean START_BACKGROUND_SERVICE = true;
    public static boolean INCLUDE_TAG_INDEX = true;
    public static long MAX_DB_SIZE_BYTES = 10 * 1024 * 1024; // 10 MB soft cap

    // === Public API ===
    public static void initialize(@NonNull Context context) {
        WatchdogEngine.getInstance().initialize(context.getApplicationContext());
        if (START_BACKGROUND_SERVICE) {
            context.getApplicationContext().startService(new Intent(context, WatchdogService.class));
        }
        if (AUTO_START) {
            enable();
        }
    }

    public static void enable() {
        WatchdogEngine.getInstance().enable();
    }

    public static void disable() {
        WatchdogEngine.getInstance().disable();
    }

    public static void pause() {
        WatchdogEngine.getInstance().pause();
    }

    public static void resume(boolean flushPendingImmediately) {
        WatchdogEngine.getInstance().resume(flushPendingImmediately);
    }

    public static void log(@NonNull String channel, @NonNull String payload) {
        WatchdogEngine.getInstance().log(channel, payload, null);
    }

    public static void log(@NonNull String channel, @NonNull String payload, String tags) {
        WatchdogEngine.getInstance().log(channel, payload, tags);
    }

    public static List<WatchdogRecord> getRecentRecords(String channel, int limit) {
        return WatchdogEngine.getInstance().getRecentRecords(channel, limit);
    }

    public static boolean isEnabled() {
        return WatchdogEngine.getInstance().isEnabled();
    }

    public static void pushStateToTelemetry(@NonNull Telemetry telemetry) {
        WatchdogEngine state = WatchdogEngine.getInstance();
        telemetry.addData("Watchdog", state.isEnabled() ? "ENABLED" : "DISABLED");
        telemetry.addData("Queue", state.getQueueDepth());
        telemetry.addData("HTTP", ENABLE_HTTP_SERVER ? "ON" : "OFF");
        telemetry.addData("Target Port", HTTP_PORT);
    }
}
