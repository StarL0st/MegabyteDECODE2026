package org.firstinspires.ftc.teamcode.base.subsystems.arcsystems;

import com.bylazar.telemetry.JoinedTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Used for the creation of the subsystem instances and binding registration.
 * <p>
 * DO NOT MODIFY.
 */
public class ARCSystemsContext {
    private final HardwareMap hardwareMap;
    private final JoinedTelemetry telemetry;
    private final GamepadEx driverOp;
    private final GamepadEx toolOp;

    public ARCSystemsContext(HardwareMap hardwareMap, JoinedTelemetry telemetry, GamepadEx driverOp, GamepadEx toolOp) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.driverOp = driverOp;
        this.toolOp = toolOp;
    }

    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    public JoinedTelemetry getTelemetry() {
        return telemetry;
    }

    public GamepadEx getDriverOp() {
        return driverOp;
    }

    public GamepadEx getToolOp() {
        return toolOp;
    }
}
