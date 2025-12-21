package com.watchdog;

import android.content.Context;

/**
 * Helper to ensure Watchdog initializes exactly once without static context hacks.
 */
final class WatchdogInitializer {
    private static Context context;

    static synchronized void setContext(Context ctx) {
        if (context == null && ctx != null) {
            context = ctx.getApplicationContext();
        }
    }

    static synchronized Context getContext() {
        return context;
    }
}
