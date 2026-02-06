package org.firstinspires.ftc.teamcode.base.constants;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class DriveConstants {
    public static double TURN_P_GAIN = 0.0022;

    // ---- AIMING ----
    public static double AIM_P_GAIN = 0.025;
    public static double AIM_D_GAIN = 0.1;
    public static double ANGLE_TOLERANCE = 0.4;

    // ---- AUTO-DRIVE ----
    public static double DISTANCE_P_GAIN = 0.2;
    public static double DISTANCE_D_GAIN = 0.075;
    public static double DISTANCE_TOLERANCE = 5;

    public static double TARGET_DIST = 115;

    public static double STRAFE_SPEED = 0.5;
    public static long STRAFE_DURATION_MS = 850;

    public static double ACQUIRE_VISION_SPEED = 0.3;
    public static long ACQUIRE_VISION_TIMEOUT = 2000;

    public static double FORWARD_STRAFE_SPEED = -0.5;
    public static long FORWARD_STRAFE_DURATION_MS = 600;
}
