package org.firstinspires.ftc.teamcode.base.command.drivetrain;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.base.PoseTracker;
import org.firstinspires.ftc.teamcode.base.subsystems.Drivetrain;

public class AcquireVisionCommand extends CommandBase {
    private final Drivetrain drivetrain;
    private final double backupSpeed;
    private final long timeoutMs;
    private long startTime;

    public AcquireVisionCommand(Drivetrain drivetrain, double backupSpeed, long timeoutMs) {
        this.drivetrain = drivetrain;
        this.backupSpeed = backupSpeed;
        this.timeoutMs = timeoutMs;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        drivetrain.setAllMotors(backupSpeed);
    }

    @Override
    public boolean isFinished() {
        return PoseTracker.INSTANCE.isTargetVisible() ||
                (System.currentTimeMillis() - startTime >= timeoutMs);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setAllMotors(0);
    }
}