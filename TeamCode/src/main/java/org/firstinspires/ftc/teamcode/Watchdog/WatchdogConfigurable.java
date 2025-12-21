package org.firstinspires.ftc.teamcode.Watchdog;

import com.watchdog.Watchdog;

/**
 * Example of customizing Watchdog configuration separate from the library package.
 * JUST AN EXAMPLE --- NOT USED ANYWHERE!
 */
public final class WatchdogConfigurable {
    private WatchdogConfigurable() {}

    public static void tuneForMatch() {
        Watchdog.HTTP_PORT = 8024;
        Watchdog.MAX_DB_SIZE_BYTES = 12 * 1024 * 1024;
        Watchdog.QUEUE_CAPACITY = 2048;
    }
}

