package org.firstinspires.ftc.teamcode.base.command.drivetrain;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.base.subsystems.Drivetrain;

public class DriveToDistanceCommand extends CommandBase {
    private final Drivetrain drivetrain;
    private final double targetDistance;

    public DriveToDistanceCommand(Drivetrain drivetrain, double targetDistance) {
        this.drivetrain = drivetrain;
        this.targetDistance = targetDistance;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.setTargetDistance(targetDistance);
    }

    @Override
    public void execute() {
        drivetrain.driveToDistance();
    }

    @Override
    public boolean isFinished() {
        return drivetrain.isAtTarget();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopDistanceControl();
    }
}