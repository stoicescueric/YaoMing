package org.firstinspires.ftc.teamcode.Hardware.Outtake;

import com.acmerobotics.dashboard.config.Config;

@Config
public class LauncherConstants {
    // Target hood tilt servo position (0-1), exposed in dashboard
    public static double target_tilt = 0.5;

    // Effective gravity in in/s^2 used for ballistic calculation
    public static double GRAVITY = 386.09; // 9.81 m/s^2 in in/s^2

    // Vertical distance from launcher muzzle to target center (inches)
    public static double TARGET_HEIGHT_DELTA_IN = 24.0;

    // Min/max launch angles in radians for the solver
    public static double MIN_LAUNCH_ANGLE_RAD = Math.toRadians(10.0);
    public static double MAX_LAUNCH_ANGLE_RAD = Math.toRadians(60.0);

    // Shooter encoder ticks per motor revolution
    public static double SHOOTER_TICKS_PER_REV = 28.0;

    // Flywheel radius in inches
    public static double FLYWHEEL_RADIUS_IN = 2.835;

    // Gear ratio from motor to flywheel (flywheel revs per motor rev)
    public static double FLYWHEEL_GEAR_RATIO = 1.0;

    // Fraction of flywheel rim speed transferred to projectile
    public static double PROJECTILE_TRANSFER_COEFF = 0.8;

    // Scale factor from ideal projectile speed to commanded RPM
    public static double PROJECTILE_SPEED_TO_RPM_SCALE = 1.0;

    // Linear mapping from launch angle (deg) to servo pos, tuned via dashboard
    public static double HOOD_SERVO_SLOPE = 1.0 / 90.0; // ~1 at 90deg
    public static double HOOD_SERVO_OFFSET = 0.0;
}
