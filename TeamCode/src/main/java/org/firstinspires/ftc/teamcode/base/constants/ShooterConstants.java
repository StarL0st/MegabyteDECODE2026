package org.firstinspires.ftc.teamcode.base.constants;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.geometry.Pose;

@Configurable
public class ShooterConstants {
    public static Pose GOAL_POS_RED = new Pose(138, 138);
    public static Pose GOAL_POS_BLUE = GOAL_POS_RED.mirror();
    public static double SCORE_HEIGHT = 26;
    public static double SCORE_ANGLE = Math.toRadians(-30);

    public static final double FLYWHEEL_OFF = 0.0;

    public static final double HOOD_LOW = 0.0;
    public static final double HOOD_HIGH = 45;

    public static double flywheelKp = 21.5;
    public static double flywheelKi = 0.5;
    public static double flywheelKd = 2;
    public static double flywheelKf = 8;

    //flywheel
    public static double targetFlywheelSpeed = 0.9;
    public static double servoTargetPos = 15;

}
