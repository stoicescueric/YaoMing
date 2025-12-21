package com.watchdog;

import androidx.annotation.NonNull;

/**
 * Immutable log payload routed through the background queue so OpModes never block.
 */
final class WatchdogEntry {
    final String channel;
    final String payload;
    final long timestamp;
    final String tags;

    WatchdogEntry(@NonNull String channel, @NonNull String payload, long timestamp, String tags) {
        this.channel = channel;
        this.payload = payload;
        this.timestamp = timestamp;
        this.tags = tags;
    }
}
