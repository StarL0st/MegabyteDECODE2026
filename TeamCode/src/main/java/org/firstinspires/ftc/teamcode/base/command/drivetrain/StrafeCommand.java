package org.firstinspires.ftc.teamcode.base.command.drivetrain;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.base.subsystems.Drivetrain;

public class StrafeCommand extends CommandBase {
    private final Drivetrain drivetrain;
    private final double strafeSpeed;
    private final double forwardSpeed;
    private final long durationMs;
    private long startTime;

    /**
     * @param strafeSpeed positive = left, negative = right
     * @param forwardSpeed positive = forward, negative = backward
     * @param durationMs how long to strafe in milliseconds
     */
    public StrafeCommand(Drivetrain drivetrain, double strafeSpeed, double forwardSpeed, long durationMs) {
        this.drivetrain = drivetrain;
        this.strafeSpeed = strafeSpeed;
        this.forwardSpeed = forwardSpeed;
        this.durationMs = durationMs;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        double y = forwardSpeed;
        double x = strafeSpeed * 1.1;
        double rx = 0;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        drivetrain.setMotorPowers(frontLeftPower, backLeftPower, frontRightPower, backRightPower);
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - startTime >= durationMs;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setAllMotors(0);
    }
}