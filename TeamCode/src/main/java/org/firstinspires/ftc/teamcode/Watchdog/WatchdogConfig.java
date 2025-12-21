package org.firstinspires.ftc.teamcode.Watchdog;

import android.content.Context;

import com.watchdog.Watchdog;

/**
 * TeamCode-side shim so teams can override Watchdog settings without touching the library package.
 */
public final class WatchdogConfig {
    private WatchdogConfig() {}

    public static void applyDefaults() {
        // override Watchdog.* fields here if desired, e.g.
        // Watchdog.HTTP_PORT = 8123;
        // Watchdog.MAX_DB_SIZE_BYTES = 20 * 1024 * 1024;
    }

    public static void initialize(Context context) {
        applyDefaults();
        Watchdog.initialize(context);
    }
}

