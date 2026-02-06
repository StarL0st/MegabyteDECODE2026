package org.firstinspires.ftc.teamcode.base;

import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

/**
 * To handle all position tracking (in PedroPathing format) & heading data,
 * also stores LL tX & tA data, utilized in distance regression calculations.
 */
public class PoseTracker {
    public static PoseTracker INSTANCE = new PoseTracker();

    public double heading;
    public double x;
    public double y;

    private DistanceUnit unit;

    public double tX;
    public double tA;

    public boolean isTargetVisible = false;

    public PoseTracker() {

    }

    /**
     * Calculates the distance of the robot based in a regression
     * made in Desmos by taking multiple setpoints and creating a
     * Power Regression (in CM).
     * @param Ta tA value from Limelight
     * @return calculated distance in CM
     */
    public static double distRegression(double Ta) {
        return 166.49119 * Math.pow(Ta, -0.590128);
    }

    public void setLLPose(Pose3D pose) {
        this.unit = pose.getPosition().unit;
        this.setX(pose.getPosition().x);
        this.setY(pose.getPosition().y);
    }

    public Pose getPedroPose() {
        return PoseConverter.pose2DToPose(new Pose2D(this.unit, this.x, this.y, AngleUnit.DEGREES, this.heading), InvertedFTCCoordinates.INSTANCE)
                .getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }

    public boolean isTargetVisible() {
        return this.isTargetVisible;
    }

    public double getHeading() {
        return heading;
    }

    public double getNormalizedHeading() {
        double normalized = (heading + 360) % 360;
        if(normalized == 360) normalized = 0;
        return normalized;
    }

    public void setHeading(double heading) {
        this.heading = heading;
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public double getTx() {
        return tX;
    }

    public double getTa() {
        return tA;
    }

    public void setTx(double tX) {
        this.tX = tX;
    }

    public void setTa(double tA) {
        this.tA = tA;
    }
}
