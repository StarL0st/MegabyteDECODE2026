package org.firstinspires.ftc.teamcode.base.command.drivetrain;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.base.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.base.subsystems.IntakeAndTransfer;

public class PulsedFeedCommand extends CommandBase {
    private final IntakeAndTransfer intakeTransfer;
    private final int numArtifacts;
    private final long feedDurationMs;
    private final long pauseDurationMs;

    private int artifactsFed = 0;
    private long stateStartTime = 0;
    private boolean isFeeding = false;

    /**
     * @param intakeTransfer the intake/transfer subsystem
     * @param numArtifacts number of artifacts to feed
     * @param feedDurationMs how long to run transfer motor per artifact (ms)
     * @param pauseDurationMs how long to pause between artifacts for flywheel recovery (ms)
     */
    public PulsedFeedCommand(IntakeAndTransfer intakeTransfer, int numArtifacts,
                             long feedDurationMs, long pauseDurationMs) {
        this.intakeTransfer = intakeTransfer;
        this.numArtifacts = numArtifacts;
        this.feedDurationMs = feedDurationMs;
        this.pauseDurationMs = pauseDurationMs;
        addRequirements(intakeTransfer);
    }

    /**
     * Convenience constructor with default timings and speed
     * @param intakeTransfer the intake/transfer subsystem
     * @param numArtifacts number of artifacts to feed
     */
    public PulsedFeedCommand(IntakeAndTransfer intakeTransfer, int numArtifacts) {
        this(intakeTransfer, numArtifacts, 500, 300); // 500ms feed, 300ms pause, full speed
    }

    @Override
    public void initialize() {
        artifactsFed = 0;
        isFeeding = true;
        stateStartTime = System.currentTimeMillis();
        // Start feeding first artifact (intake off, transfer on)
        intakeTransfer.setMotorSpeed(IntakeConstants.TRANSFER_MOTOR_POWER);
    }

    @Override
    public void execute() {
        long elapsed = System.currentTimeMillis() - stateStartTime;

        if (isFeeding) {
            // Currently feeding an artifact
            if (elapsed >= feedDurationMs) {
                // Done feeding this artifact, start pause
                intakeTransfer.setMotorSpeed(0.0);
                artifactsFed++;
                isFeeding = false;
                stateStartTime = System.currentTimeMillis();
            }
        } else {
            // Currently pausing for flywheel recovery
            if (elapsed >= pauseDurationMs) {
                // Done pausing, check if we need to feed more
                if (artifactsFed < numArtifacts) {
                    // Feed next artifact
                    intakeTransfer.setMotorSpeed(IntakeConstants.TRANSFER_MOTOR_POWER);
                    isFeeding = true;
                    stateStartTime = System.currentTimeMillis();
                }
            }
        }
    }

    @Override
    public boolean isFinished() {
        return artifactsFed >= numArtifacts && !isFeeding;
    }

    @Override
    public void end(boolean interrupted) {
        intakeTransfer.stop();
    }
}