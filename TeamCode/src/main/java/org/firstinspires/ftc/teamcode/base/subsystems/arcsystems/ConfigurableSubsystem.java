package org.firstinspires.ftc.teamcode.base.subsystems.arcsystems;

import com.seattlesolvers.solverslib.command.Command;

public interface ConfigurableSubsystem {

    Command createDefaultCommand(ARCSystemsContext ctx);

    default void configureBindings(ARCSystemsContext ctx) {
        //no op
    }
}
