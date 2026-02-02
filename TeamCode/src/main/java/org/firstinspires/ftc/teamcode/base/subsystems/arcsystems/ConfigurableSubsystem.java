package org.firstinspires.ftc.teamcode.base.subsystems.arcsystems;

import com.seattlesolvers.solverslib.command.Command;

/**
 * Interface to allow dynamic binding of controls and default commands in subsystems.
 * <p>
 * DO NOT MODIFY.
 */
public interface ConfigurableSubsystem {

    Command createDefaultCommand(ARCSystemsContext ctx);

    default void configureBindings(ARCSystemsContext ctx) {
        //no op
    }
}
