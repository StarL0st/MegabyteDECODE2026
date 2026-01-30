package org.firstinspires.ftc.teamcode.base.constants;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.geometry.Pose;

public class ShooterConstants {
    public static Pose GOAL_POS_RED = new Pose(138, 138);
    public static Pose GOAL_POS_BLUE = GOAL_POS_RED.mirror();
    public static double SCORE_HEIGHT = 26;
    public static double SCORE_ANGLE = Math.toRadians(-30);

    public static final double FLYWHEEL_OFF = 0.0;

    public static final double HOOD_LOW = 0.0;

    public static PIDFCoefficients FLYWHEEL_PIDF = new PIDFCoefficients(0.006, 0.0015, 0, 0.091);
}
