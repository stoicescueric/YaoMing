package com.watchdog;

import android.content.Context;
import android.util.Log;

import java.lang.reflect.Method;

/**
 * Provides a Context for components that require it before Watchdog initialization; falls back gracefully.
 */
final class WatchdogContextProvider {
    Context get() {
        try {
            Class<?> appClass = Class.forName("com.qualcomm.robotcore.app.RobotCoreApplication");
            Method getInstance = appClass.getMethod("getInstance");
            Object app = getInstance.invoke(null);
            if (app instanceof Context) {
                return (Context) app;
            }
        } catch (Exception e) {
            Log.w("Watchdog", "Unable to obtain RobotCoreApplication context, using null", e);
        }
        return null;
    }
}
