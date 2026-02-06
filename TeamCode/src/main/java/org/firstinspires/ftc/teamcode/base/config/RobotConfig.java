package org.firstinspires.ftc.teamcode.base.config;

/**
 * Configuration data for alliance and starting position
 */
public class RobotConfig {
    public enum Alliance {
        RED,
        BLUE
    }

    public enum StartPosition {
        FAR_SIDE,
        CLOSE_SIDE
    }

    private Alliance alliance;
    private StartPosition startPosition;

    // Default constructor for GSON
    public RobotConfig() {
        this.alliance = Alliance.RED;
        this.startPosition = StartPosition.CLOSE_SIDE;
    }

    public RobotConfig(Alliance alliance, StartPosition startPosition) {
        this.alliance = alliance;
        this.startPosition = startPosition;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    public StartPosition getStartPosition() {
        return startPosition;
    }

    public void setStartPosition(StartPosition startPosition) {
        this.startPosition = startPosition;
    }

    @Override
    public String toString() {
        return "Alliance: " + alliance + ", Start Position: " + startPosition;
    }
}