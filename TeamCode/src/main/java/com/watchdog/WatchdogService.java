package com.watchdog;

import android.app.Service;
import android.content.Intent;
import android.os.IBinder;
import android.util.Log;

import androidx.annotation.Nullable;

/**
 * Android Service hosting the background queue, SQLite DB, and HTTP server.
 */
public class WatchdogService extends Service {
    private static final String TAG = "WatchdogService";

    @Override
    public void onCreate() {
        super.onCreate();
        Log.i(TAG, "WatchdogService created");
        WatchdogEngine.getInstance().initialize(getApplicationContext());
    }

    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {
        WatchdogEngine.getInstance().enable();
        return START_STICKY;
    }

    @Nullable
    @Override
    public IBinder onBind(Intent intent) {
        return null;
    }

    @Override
    public void onDestroy() {
        WatchdogEngine.getInstance().disable();
        super.onDestroy();
        Log.i(TAG, "WatchdogService destroyed");
    }
}
