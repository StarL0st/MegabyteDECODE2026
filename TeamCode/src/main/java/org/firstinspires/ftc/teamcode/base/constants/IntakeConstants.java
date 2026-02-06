package org.firstinspires.ftc.teamcode.base.constants;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class IntakeConstants {
    public static double TURRET_STOP_OPEN = 26;
    public static double TURRET_STOP_CLOSE = 1;

    public static double TRANSFER_MOTOR_POWER = 0.8;
    public static double INTAKE_MOTOR_POWER = 0.9;

    public static double TRANSFER_REVERSE_POWER = -0.6;

    public static int TRANSFER_ARTIFACT_AMOUNT = 3;
    public static long TRANSFER_FEED_DURATION_MS = 350;
    public static long TRANSFER_PAUSE_DURATION_MS = 2000;
}
