package com.watchdog;

/**
 * Lightweight DTO returned when querying Watchdog logs.
 */
public final class WatchdogRecord {
    public final long id;
    public final long timestamp;
    public final String channel;
    public final String payload;
    public final String tags;

    WatchdogRecord(long id, long timestamp, String channel, String payload, String tags) {
        this.id = id;
        this.timestamp = timestamp;
        this.channel = channel;
        this.payload = payload;
        this.tags = tags;
    }
}
