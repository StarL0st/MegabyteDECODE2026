package org.firstinspires.ftc.teamcode.base.subsystems.arcsystems;

import com.bylazar.telemetry.JoinedTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

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
