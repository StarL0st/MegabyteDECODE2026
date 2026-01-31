package org.firstinspires.ftc.teamcode.base;

public class PoseTracker {
    public static PoseTracker INSTANCE = new PoseTracker();

    public double heading;
    public double x;
    public double y;

    public double tX;
    public double tA;

    public PoseTracker() {

    }

    public static double distRegression(double Ta) {
        return 166.49119 * Math.pow(Ta, -0.590128);
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
