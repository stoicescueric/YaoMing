# Watchdog Library

Watchdog is a lightweight FTC logging and observability service that runs alongside the Control Hub OS without touching the two OpMode threads. You can ship it just like Pedro Pathing: the core code lives in `com.watchdog`, while teams override behavior from TeamCode via `WatchdogConfig` (or your own shim).

---
## Feature Highlights
- **Non-blocking logging:** OpModes call `Watchdog.log()` which enqueues entries in constant time; a background worker drains the queue to SQLite.
- **Persistent storage:** Data is stored in `watchdog.db` inside the app’s private storage, surviving reboots and accessible via ADB or download.
- **Live HTTP dashboard:** Port `8024` serves a responsive UI plus JSON APIs and a raw DB download endpoint.
- **Auto-start on boot:** A boot receiver launches the `WatchdogService` whenever the Control Hub powers up or the APK updates, so logging is always ready.
- **Pause/resume controls:** OpModes can temporarily halt processing (e.g., mid-match) and flush queued data on resume.
- **Configurable via TeamCode:** Override ports, queue sizes, batch intervals, DB caps, etc., without forking the library.
- **Tiny footprint:** A single worker thread plus a socket listener, keeping cycle time untouched.

---
## Architecture Overview
- **Watchdog (API):** exposes static configuration fields, `initialize/enable/disable/pause/resume/log/getRecentRecords`, and telemetry helpers.
- **WatchdogEngine:** owns the queue, batching loop, SQLite helper, and HTTP server lifecycle.
- **WatchdogService:** Android `Service` that starts the engine; marked `START_STICKY` so Android restarts it if killed.
- **WatchdogBootReceiver:** listens for `BOOT_COMPLETED` and `MY_PACKAGE_REPLACED` to auto-launch the service.
- **WatchdogDatabaseHelper:** handles schema (`logs` table + channel index), size enforcement, JSON conversion.
- **WatchdogHttpServer:** lightweight socket server with `/`, `/api/logs`, `/download` routes backed by the built-in HTML/JS from `WatchdogAssets`.

---
## Quick Start (TeamCode)
```java
// org.firstinspires.ftc.teamcode.Watchdog.WatchdogConfig.java
public final class WatchdogConfig {
    private WatchdogConfig() {}

    public static void applyDefaults() {
        Watchdog.HTTP_PORT = 8024;
        Watchdog.MAX_DB_SIZE_BYTES = 12 * 1024 * 1024;
        Watchdog.QUEUE_CAPACITY = 2048;
    }

    public static void initialize(Context context) {
        applyDefaults();
        Watchdog.initialize(context);
    }
}
```
Use it inside an OpMode:
```java
@Override
public void init() {
    WatchdogConfig.initialize(this);
}

@Override
public void loop() {
    Watchdog.log("teleop", "driveForward=" + gamepad1.left_stick_y, "telemetry");
    Watchdog.pushStateToTelemetry(telemetry);
    telemetry.update();
}
```
> Note: even if you never call `initialize`, the service still runs on boot; `initialize()` just lets you tweak config at runtime or display telemetry.

---
## HTTP & CLI Usage
| Path | Description |
|------|-------------|
| `/` | Live dashboard with channel/limit filters, auto-refreshing table, download link |
| `/api/logs?channel=intake&limit=200` | JSON payload of recent rows |
| `/download` | Streams `watchdog.db` for archival or offline analysis |

ADB commands:
```bash
adb connect <hub-ip>
adb shell run-as com.qualcomm.ftcrobotcontroller ls databases
adb shell run-as com.qualcomm.ftcrobotcontroller cat databases/watchdog.db > watchdog.db
```

---
## Configuration Reference
All knobs live on `com.watchdog.Watchdog`:
- `HTTP_PORT` – default `8024`
- `QUEUE_CAPACITY` – max in-memory entries (default `1024`)
- `MAX_BATCH_SIZE` – inserts per SQLite transaction (`32`)
- `BATCH_FLUSH_INTERVAL_MS` – sleep when queue empty (`25` ms)
- `DROP_WHEN_QUEUE_FULL` – drop newest vs. evict oldest when full
- `MAX_DB_SIZE_BYTES` – soft cap; trims oldest 10% when exceeded
- `AUTO_START`, `START_BACKGROUND_SERVICE`, `ENABLE_HTTP_SERVER` – feature flags
- `PAUSE_POLL_INTERVAL_MS` – how often the worker wakes while paused

Sample override for a data-heavy testing session:
```java
public static void applyDefaults() {
    Watchdog.HTTP_PORT = 9000;
    Watchdog.MAX_DB_SIZE_BYTES = 50L * 1024 * 1024;
    Watchdog.QUEUE_CAPACITY = 4096;
    Watchdog.MAX_BATCH_SIZE = 64;
    Watchdog.AUTO_START = false; // let us toggle manually mid-test
}
```
Then elsewhere:
```java
if (gamepad1.start && gamepad1.back) {
    Watchdog.enable();
}
if (gamepad1.left_bumper) {
    Watchdog.pause();
}
if (gamepad1.right_bumper) {
    Watchdog.resume(true); // flush before resuming
}
```

---
## Data Model & Query Helpers
SQLite schema:
```sql
CREATE TABLE logs (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp INTEGER NOT NULL,
    channel TEXT NOT NULL,
    payload TEXT NOT NULL,
    tags TEXT
);
CREATE INDEX idx_channel ON logs(channel);
```
Programmatic access:
```java
List<WatchdogRecord> latest = Watchdog.getRecentRecords("intake", 20);
for (WatchdogRecord row : latest) {
    telemetry.addLine(row.channel + " > " + row.payload);
}
```
`WatchdogRecord` exposes `id`, `timestamp`, `channel`, `payload`, `tags`.

---
## Boot & Lifecycle Notes
1. When the Control Hub boots, Android fires `BOOT_COMPLETED` → `WatchdogBootReceiver` starts `WatchdogService`.
2. The service initializes the engine if needed and calls `enable()`.
3. If Android kills the service, `START_STICKY` restarts it automatically.
4. `disable()` stops the HTTP server and halts queue draining; you can re-enable at any time.

---
## Troubleshooting
| Symptom | Check |
|---------|-------|
| HTTP site not reachable | Ensure Wi-Fi is connected, verify `Watchdog.HTTP_PORT`, confirm service running via `adb shell ps | grep Watchdog` |
| Logs missing | Confirm `Watchdog.log()` is called (channel name spelled correctly); inspect `/api/logs?limit=10` |
| DB grows too large | Lower `MAX_DB_SIZE_BYTES` or call `DROP_WHEN_QUEUE_FULL = true` |
| Need to disable temporarily | Call `Watchdog.disable()` or set `START_BACKGROUND_SERVICE = false` before deployment |

---
## Demo OpMode
`org.firstinspires.ftc.teamcode.Watchdog.WatchdogDemoOpMode` periodically logs uptime, demonstrates telemetry integration, and can be cloned for your own testing.

```
