package com.watchdog;

import android.content.Context;
import android.database.Cursor;
import android.util.Log;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.URLDecoder;
import java.nio.charset.StandardCharsets;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Lightweight HTTP server exposing Watchdog data via JSON + HTML without blocking FTC threads.
 */
final class WatchdogHttpServer {
    private static final String TAG = "WatchdogHttp";
    private final Context context;
    private final WatchdogDatabaseHelper dbHelper;
    private final ExecutorService clientPool = Executors.newCachedThreadPool();
    private final AtomicBoolean running = new AtomicBoolean(false);
    private ServerSocket serverSocket;
    private Thread acceptThread;

    WatchdogHttpServer(Context context, WatchdogDatabaseHelper dbHelper) {
        this.context = context.getApplicationContext();
        this.dbHelper = dbHelper;
    }

    void start(int port) {
        if (running.get()) return;
        try {
            serverSocket = new ServerSocket(port);
            running.set(true);
            acceptThread = new Thread(this::acceptLoop, "WatchdogHttpAccept");
            acceptThread.start();
            Log.i(TAG, "HTTP server listening on port " + port);
        } catch (IOException e) {
            Log.e(TAG, "Unable to start HTTP server", e);
        }
    }

    void stop() {
        running.set(false);
        if (serverSocket != null) {
            try {
                serverSocket.close();
            } catch (IOException ignored) {}
        }
        if (acceptThread != null) {
            acceptThread.interrupt();
            acceptThread = null;
        }
        clientPool.shutdownNow();
    }

    private void acceptLoop() {
        while (running.get()) {
            try {
                Socket client = serverSocket.accept();
                clientPool.submit(() -> handleClient(client));
            } catch (IOException e) {
                if (running.get()) {
                    Log.e(TAG, "HTTP accept failed", e);
                }
            }
        }
    }

    private void handleClient(Socket socket) {
        try {
            BufferedReader reader = new BufferedReader(new InputStreamReader(socket.getInputStream(), StandardCharsets.UTF_8));
            OutputStream out = socket.getOutputStream();
            String requestLine = reader.readLine();
            if (requestLine == null || requestLine.isEmpty()) {
                respondText(out, 400, "Bad Request", "Missing request line");
                socket.close();
                return;
            }
            String[] parts = requestLine.split(" ");
            if (parts.length < 3) {
                respondText(out, 400, "Bad Request", "Malformed request");
                socket.close();
                return;
            }
            String method = parts[0];
            String fullPath = parts[1];
            int qIndex = fullPath.indexOf('?');
            String path = qIndex >= 0 ? fullPath.substring(0, qIndex) : fullPath;
            String query = qIndex >= 0 ? fullPath.substring(qIndex + 1) : null;

            String line;
            while ((line = reader.readLine()) != null && !line.isEmpty()) {
                // ignore headers for now
            }

            if (!"GET".equalsIgnoreCase(method)) {
                respondText(out, 405, "Method Not Allowed", "Only GET supported");
                socket.close();
                return;
            }

            switch (path) {
                case "/":
                    respondHtml(out, WatchdogAssets.indexHtml());
                    break;
                case "/api/logs":
                    handleLogs(out, query);
                    break;
                case "/download":
                    handleDownload(out);
                    break;
                case "/logo.svg":
                    handleLogo(out);
                    break;
                case "/api/pose/runs":
                    handlePoseRuns(out);
                    break;
                case "/api/pose/export":
                    handlePoseExport(out, query);
                    break;
                case "/api/pose/import":
                    // simplified: expect a file already pushed on-disk; in-browser upload would
                    // need POST/multipart which is out of scope for this tiny server.
                    respondText(out, 501, "Not Implemented", "Upload via adb not implemented in HTTP");
                    break;
                default:
                    respondText(out, 404, "Not Found", "Unknown path");
            }
            socket.close();
        } catch (IOException e) {
            Log.e(TAG, "Error handling HTTP client", e);
        }
    }

    private void handleLogs(OutputStream out, String query) throws IOException {
        Map<String, String> params = splitQuery(query);
        String channel = params.get("channel");
        int limit = parseInt(params.get("limit"), 200);
        Cursor cursor = WatchdogDatabaseHelper.queryLogs(dbHelper.getReadableDatabase(), channel, limit);
        String json = WatchdogDatabaseHelper.cursorToJson(cursor);
        respond(out, 200, "OK", "application/json", json.getBytes(StandardCharsets.UTF_8));
    }

    private void handleDownload(OutputStream out) throws IOException {
        File dbFile = context.getDatabasePath(WatchdogDatabaseHelper.DB_NAME);
        if (dbFile == null || !dbFile.exists()) {
            respondText(out, 404, "Not Found", "Database not found");
            return;
        }
        byte[] header = ("HTTP/1.1 200 OK\r\n" +
                "Content-Type: application/octet-stream\r\n" +
                "Content-Disposition: attachment; filename=watchdog.db\r\n" +
                "Content-Length: " + dbFile.length() + "\r\n" +
                "Connection: close\r\n\r\n").getBytes(StandardCharsets.UTF_8);
        out.write(header);
        try (FileInputStream fis = new FileInputStream(dbFile)) {
            byte[] buffer = new byte[8192];
            int read;
            while ((read = fis.read(buffer)) != -1) {
                out.write(buffer, 0, read);
            }
        }
        out.flush();
    }

    private void handleLogo(OutputStream out) throws IOException {
        File svgFile = new File(context.getFilesDir().getParentFile(),
                "app_TeamCode/src/main/java/com/watchdog/WATCHDOG.svg");
        if (!svgFile.exists()) {
            respondText(out, 404, "Not Found", "Logo asset not found");
            return;
        }
        byte[] header = ("HTTP/1.1 200 OK\r\n" +
                "Content-Type: image/svg+xml\r\n" +
                "Connection: close\r\n\r\n").getBytes(StandardCharsets.UTF_8);
        out.write(header);
        try (FileInputStream fis = new FileInputStream(svgFile)) {
            byte[] buffer = new byte[4096];
            int read;
            while ((read = fis.read(buffer)) != -1) {
                out.write(buffer, 0, read);
            }
        }
        out.flush();
    }

    /**
     * Returns a compact list of pose runs derived from pose-channel logs.
     * JSON array of objects: { runId, firstTimestamp, lastTimestamp, count }
     */
    private void handlePoseRuns(OutputStream out) throws IOException {
        Cursor cursor = WatchdogDatabaseHelper.queryLogs(dbHelper.getReadableDatabase(), "pose", 10000);
        JSONArray array = new JSONArray();
        if (cursor != null) {
            try {
                // Map runId -> [firstTs, lastTs, count]
                Map<String, long[]> runs = new LinkedHashMap<>();
                while (cursor.moveToNext()) {
                    String payload = cursor.getString(cursor.getColumnIndexOrThrow("payload"));
                    if (payload == null) continue;
                    String[] parts = payload.split(",");
                    if (parts.length < 5) continue;
                    String runId = parts[0];
                    long ts;
                    try {
                        ts = Long.parseLong(parts[1]);
                    } catch (NumberFormatException e) {
                        continue;
                    }
                    long[] agg = runs.get(runId);
                    if (agg == null) {
                        agg = new long[]{ts, ts, 0};
                        runs.put(runId, agg);
                    } else {
                        if (ts < agg[0]) agg[0] = ts;
                        if (ts > agg[1]) agg[1] = ts;
                    }
                    agg[2]++;
                }
                for (Map.Entry<String, long[]> e : runs.entrySet()) {
                    JSONObject obj = new JSONObject();
                    obj.put("runId", e.getKey());
                    obj.put("firstTimestamp", e.getValue()[0]);
                    obj.put("lastTimestamp", e.getValue()[1]);
                    obj.put("count", e.getValue()[2]);
                    array.put(obj);
                }
            } catch (Exception ex) {
                Log.e(TAG, "handlePoseRuns error", ex);
            } finally {
                cursor.close();
            }
        }
        byte[] body = array.toString().getBytes(StandardCharsets.UTF_8);
        respond(out, 200, "OK", "application/json", body);
    }

    /**
     * Export a single pose run as a .wd JSON file. Query params: runId=...  (required)
     */
    private void handlePoseExport(OutputStream out, String query) throws IOException {
        Map<String, String> params = splitQuery(query);
        String runId = params.get("runId");
        if (runId == null || runId.isEmpty()) {
            respondText(out, 400, "Bad Request", "runId required");
            return;
        }
        // Collect pose-channel logs for this runId
        Cursor cursor = WatchdogDatabaseHelper.queryLogs(dbHelper.getReadableDatabase(), "pose", 10000);
        JSONArray array = new JSONArray();
        if (cursor != null) {
            try {
                while (cursor.moveToNext()) {
                    String payload = cursor.getString(cursor.getColumnIndexOrThrow("payload"));
                    if (payload == null) continue;
                    String[] parts = payload.split(",");
                    if (parts.length < 5) continue;
                    if (!runId.equals(parts[0])) continue;
                    JSONObject obj = new JSONObject();
                    obj.put("timestamp", Long.parseLong(parts[1]));
                    obj.put("x", Double.parseDouble(parts[2]));
                    obj.put("y", Double.parseDouble(parts[3]));
                    obj.put("heading", Double.parseDouble(parts[4]));
                    array.put(obj);
                }
            } catch (Exception e) {
                Log.e(TAG, "handlePoseExport error", e);
            } finally {
                cursor.close();
            }
        }
        JSONObject root = new JSONObject();
        try {
            root.put("runId", runId);
            root.put("points", array);
        } catch (Exception ignore) {}
        byte[] body = root.toString().getBytes(StandardCharsets.UTF_8);
        String filename = runId + ".wd";
        BufferedWriter writer = new BufferedWriter(new OutputStreamWriter(out, StandardCharsets.UTF_8));
        writer.write("HTTP/1.1 200 OK\r\n");
        writer.write("Content-Type: application/json\r\n");
        writer.write("Content-Disposition: attachment; filename=" + filename + "\r\n");
        writer.write("Content-Length: " + body.length + "\r\n");
        writer.write("Connection: close\r\n\r\n");
        writer.flush();
        out.write(body);
        out.flush();
    }

    private void respondHtml(OutputStream out, String html) throws IOException {
        respond(out, 200, "OK", "text/html; charset=utf-8", html.getBytes(StandardCharsets.UTF_8));
    }

    private void respondText(OutputStream out, int code, String status, String text) throws IOException {
        respond(out, code, status, "text/plain; charset=utf-8", text.getBytes(StandardCharsets.UTF_8));
    }

    private void respond(OutputStream out, int code, String status, String contentType, byte[] body) throws IOException {
        BufferedWriter writer = new BufferedWriter(new OutputStreamWriter(out, StandardCharsets.UTF_8));
        writer.write("HTTP/1.1 " + code + " " + status + "\r\n");
        writer.write("Content-Type: " + contentType + "\r\n");
        writer.write("Content-Length: " + body.length + "\r\n");
        writer.write("Connection: close\r\n\r\n");
        writer.flush();
        out.write(body);
        out.flush();
    }

    private Map<String, String> splitQuery(String query) {
        Map<String, String> map = new LinkedHashMap<>();
        if (query == null || query.isEmpty()) return map;
        String[] pairs = query.split("&");
        for (String pair : pairs) {
            int idx = pair.indexOf('=');
            String key = idx >= 0 ? pair.substring(0, idx) : pair;
            String value = idx >= 0 ? pair.substring(idx + 1) : "";
            try {
                key = URLDecoder.decode(key, "UTF-8");
                value = URLDecoder.decode(value, "UTF-8");
            } catch (Exception ignored) {}
            map.put(key, value);
        }
        return map;
    }

    private int parseInt(String value, int fallback) {
        try {
            return value == null ? fallback : Integer.parseInt(value);
        } catch (NumberFormatException e) {
            return fallback;
        }
    }
}
