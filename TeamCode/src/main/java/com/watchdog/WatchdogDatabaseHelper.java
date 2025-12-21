package com.watchdog;

import android.content.ContentValues;
import android.content.Context;
import android.database.Cursor;
import android.database.sqlite.SQLiteDatabase;
import android.database.sqlite.SQLiteOpenHelper;
import android.util.Log;

import androidx.annotation.Nullable;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.File;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

/**
 * Manages the SQLite DB storing Watchdog logs.
 */
final class WatchdogDatabaseHelper extends SQLiteOpenHelper {
    static final String DB_NAME = "watchdog.db";
    private static final int DB_VERSION = 1;
    private static WatchdogDatabaseHelper INSTANCE;

    static synchronized WatchdogDatabaseHelper getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new WatchdogDatabaseHelper();
        }
        return INSTANCE;
    }

    private Context context;

    void init(Context ctx) {
        if (context != null) return;
        context = ctx.getApplicationContext();
        WatchdogInitializer.setContext(context);
        SQLiteDatabase unused = getWritableDatabase();
    }

    private WatchdogDatabaseHelper() {
        super(obtainContext(), DB_NAME, null, DB_VERSION);
    }

    private static Context obtainContext() {
        Context ctx = WatchdogInitializer.getContext();
        if (ctx == null) {
            ctx = new WatchdogContextProvider().get();
            if (ctx != null) {
                WatchdogInitializer.setContext(ctx);
            }
        }
        return ctx;
    }

    @Override
    public void onConfigure(SQLiteDatabase db) {
        super.onConfigure(db);
        db.enableWriteAheadLogging();
    }

    @Override
    public void onCreate(SQLiteDatabase db) {
        db.execSQL("CREATE TABLE IF NOT EXISTS logs (" +
                "id INTEGER PRIMARY KEY AUTOINCREMENT, " +
                "timestamp INTEGER NOT NULL, " +
                "channel TEXT NOT NULL, " +
                "payload TEXT NOT NULL, " +
                "tags TEXT)");
        db.execSQL("CREATE INDEX IF NOT EXISTS idx_channel ON logs(channel)");
    }

    @Override
    public void onUpgrade(SQLiteDatabase db, int oldVersion, int newVersion) {
        // Future migrations here
    }

    void insertBatch(Collection<WatchdogEntry> batch) {
        if (batch.isEmpty()) return;
        SQLiteDatabase db = getWritableDatabase();
        db.beginTransaction();
        try {
            for (WatchdogEntry entry : batch) {
                ContentValues values = new ContentValues();
                values.put("timestamp", entry.timestamp);
                values.put("channel", entry.channel);
                values.put("payload", entry.payload);
                values.put("tags", entry.tags);
                db.insert("logs", null, values);
            }
            db.setTransactionSuccessful();
        } finally {
            db.endTransaction();
        }
        enforceSizeCap(db);
    }

    private void enforceSizeCap(SQLiteDatabase db) {
        if (context == null) return;
        File dbFile = context.getDatabasePath(DB_NAME);
        if (dbFile != null && dbFile.exists() && dbFile.length() > Watchdog.MAX_DB_SIZE_BYTES) {
            long rowsToDelete = Math.max(1, (long) (getRowCount(db) * 0.1));
            db.execSQL("DELETE FROM logs WHERE id IN (SELECT id FROM logs ORDER BY id ASC LIMIT " + rowsToDelete + ")");
            Log.w("WatchdogDB", "Trimmed " + rowsToDelete + " rows to enforce size cap");
        }
    }

    private long getRowCount(SQLiteDatabase db) {
        Cursor cursor = db.rawQuery("SELECT COUNT(*) FROM logs", null);
        try {
            return cursor.moveToFirst() ? cursor.getLong(0) : 0;
        } finally {
            cursor.close();
        }
    }

    static Cursor queryLogs(SQLiteDatabase db, @Nullable String channel, int limit) {
        String selection = channel == null ? null : "channel = ?";
        String[] selectionArgs = channel == null ? null : new String[]{channel};
        return db.query("logs", null, selection, selectionArgs, null, null, "id DESC", String.valueOf(limit));
    }

    static List<WatchdogRecord> consumeCursor(Cursor cursor) {
        List<WatchdogRecord> list = new ArrayList<>();
        try {
            while (cursor.moveToNext()) {
                list.add(new WatchdogRecord(
                        cursor.getLong(cursor.getColumnIndexOrThrow("id")),
                        cursor.getLong(cursor.getColumnIndexOrThrow("timestamp")),
                        cursor.getString(cursor.getColumnIndexOrThrow("channel")),
                        cursor.getString(cursor.getColumnIndexOrThrow("payload")),
                        cursor.getString(cursor.getColumnIndexOrThrow("tags"))
                ));
            }
        } finally {
            cursor.close();
        }
        return list;
    }

    static String cursorToJson(Cursor cursor) {
        JSONArray array = new JSONArray();
        try {
            while (cursor.moveToNext()) {
                JSONObject obj = new JSONObject();
                obj.put("id", cursor.getLong(cursor.getColumnIndexOrThrow("id")));
                obj.put("timestamp", cursor.getLong(cursor.getColumnIndexOrThrow("timestamp")));
                obj.put("channel", cursor.getString(cursor.getColumnIndexOrThrow("channel")));
                obj.put("payload", cursor.getString(cursor.getColumnIndexOrThrow("payload")));
                obj.put("tags", cursor.getString(cursor.getColumnIndexOrThrow("tags")));
                array.put(obj);
            }
        } catch (Exception e) {
            Log.e("WatchdogDB", "cursorToJson error", e);
        } finally {
            cursor.close();
        }
        return array.toString();
    }
}
