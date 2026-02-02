package org.firstinspires.ftc.teamcode.base.command.drivetrain;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.base.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

/**
 * Default DriveCommand for the Drivetrain Subsystem
 * <p>
 * Used to pass the driver 1 controller inputs to the drivetrain movement calculations.
 * Uses the SolversLib command system.
 * <p>
 * Do NOT modify.
 */
public class DriveCommand extends CommandBase {

    private final Drivetrain m_drive;
    private final DoubleSupplier m_strafe;
    private final DoubleSupplier m_forward;
    private final DoubleSupplier m_rotation;

    public DriveCommand(Drivetrain mDrive,
                        DoubleSupplier mStrafe, DoubleSupplier mForward, DoubleSupplier mRotation) {
        m_drive = mDrive;
        m_strafe = mStrafe;
        m_forward = mForward;
        m_rotation = mRotation;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        m_drive.driveWithController(m_strafe.getAsDouble(), m_forward.getAsDouble(), m_rotation.getAsDouble());
    }
}
