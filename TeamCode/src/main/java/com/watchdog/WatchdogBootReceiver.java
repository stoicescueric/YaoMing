package com.watchdog;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.util.Log;

/**
 * Ensures WatchdogService launches automatically when the Control Hub boots or the app updates.
 */
public class WatchdogBootReceiver extends BroadcastReceiver {
    private static final String TAG = "WatchdogBootReceiver";

    @Override
    public void onReceive(Context context, Intent intent) {
        if (intent == null || intent.getAction() == null) {
            return;
        }
        String action = intent.getAction();
        if (Intent.ACTION_BOOT_COMPLETED.equals(action) || Intent.ACTION_MY_PACKAGE_REPLACED.equals(action)) {
            Context appContext = context.getApplicationContext();
            try {
                appContext.startService(new Intent(appContext, WatchdogService.class));
            } catch (Exception e) {
                Log.e(TAG, "Failed to start WatchdogService on boot", e);
            }
        }
    }
}
