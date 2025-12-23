package com.watchdog;

import android.content.Context;
import android.database.sqlite.SQLiteDatabase;
import android.os.SystemClock;
import android.util.Log;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Core singleton that owns queues, database helper, and HTTP server.
 */
final class WatchdogEngine {
    private static final String TAG = "WatchdogEngine";
    private static WatchdogEngine INSTANCE;

    static synchronized WatchdogEngine getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new WatchdogEngine();
        }
        return INSTANCE;
    }

    private final AtomicBoolean initialized = new AtomicBoolean(false);
    private final AtomicBoolean enabled = new AtomicBoolean(false);
    private final AtomicBoolean paused = new AtomicBoolean(false);
    private Context appContext;
    private BlockingQueue<WatchdogEntry> queue;
    private ExecutorService dbExecutor;
    private WatchdogHttpServer httpServer;

    private WatchdogEngine() {}

    void initialize(@NonNull Context context) {
        if (initialized.get()) return;
        appContext = context.getApplicationContext();
        WatchdogInitializer.setContext(appContext);
        queue = new ArrayBlockingQueue<>(Watchdog.QUEUE_CAPACITY);
        dbExecutor = Executors.newSingleThreadExecutor();
        WatchdogDatabaseHelper.getInstance().init(appContext);
        if (Watchdog.ENABLE_HTTP_SERVER) {
            httpServer = new WatchdogHttpServer(appContext, WatchdogDatabaseHelper.getInstance());
            httpServer.start(Watchdog.HTTP_PORT);
        }
        initialized.set(true);
    }

    void enable() {
        if (!initialized.get()) {
            Log.w(TAG, "enable() called before initialize()");
            return;
        }
        if (enabled.compareAndSet(false, true)) {
            paused.set(false);
            dbExecutor.submit(this::drainQueueLoop);
        }
    }

    void disable() {
        if (enabled.compareAndSet(true, false)) {
            paused.set(false);
        }
        if (httpServer != null) {
            httpServer.stop();
        }
    }

    void pause() {
        paused.set(true);
    }

    void resume(boolean flushNow) {
        paused.set(false);
        if (flushNow) {
            dbExecutor.submit(this::flushCurrentQueue);
        }
    }

    void log(@NonNull String channel, @NonNull String payload, @Nullable String tags) {
        if (!initialized.get()) {
            Log.w(TAG, "log() called before initialize()");
            return;
        }
        WatchdogEntry entry = new WatchdogEntry(channel, payload, System.currentTimeMillis(), tags);
        if (!queue.offer(entry) && !Watchdog.DROP_WHEN_QUEUE_FULL) {
            queue.poll();
            queue.offer(entry);
        }
    }

    List<WatchdogRecord> getRecentRecords(String channel, int limit) {
        SQLiteDatabase db = WatchdogDatabaseHelper.getInstance().getReadableDatabase();
        return WatchdogDatabaseHelper.consumeCursor(WatchdogDatabaseHelper.queryLogs(db, channel, limit));
    }

    boolean isEnabled() {
        return enabled.get();
    }

    int getQueueDepth() {
        return queue == null ? 0 : queue.size();
    }

    private void drainQueueLoop() {
        List<WatchdogEntry> batch = new ArrayList<>(Watchdog.MAX_BATCH_SIZE);
        while (enabled.get()) {
            try {
                if (paused.get()) {
                    SystemClock.sleep(Watchdog.PAUSE_POLL_INTERVAL_MS);
                    continue;
                }
                WatchdogEntry first = queue.poll();
                if (first == null) {
                    SystemClock.sleep(Watchdog.BATCH_FLUSH_INTERVAL_MS);
                    continue;
                }
                batch.clear();
                batch.add(first);
                queue.drainTo(batch, Watchdog.MAX_BATCH_SIZE - 1);
                WatchdogDatabaseHelper.getInstance().insertBatch(batch);
            } catch (Exception e) {
                Log.e(TAG, "Error draining Watchdog queue", e);
            }
        }
        flushCurrentQueue();
    }

    private void flushCurrentQueue() {
        if (queue == null || queue.isEmpty()) {
            return;
        }
        List<WatchdogEntry> batch = new ArrayList<>(queue.size());
        queue.drainTo(batch);
        if (!batch.isEmpty()) {
            WatchdogDatabaseHelper.getInstance().insertBatch(batch);
        }
    }
}
