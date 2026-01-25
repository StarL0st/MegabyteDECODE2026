package org.firstinspires.ftc.teamcode.base;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.command.drivetrain.DriveCommand;
import org.firstinspires.ftc.teamcode.base.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.base.subsystems.Intake;
import org.firstinspires.ftc.teamcode.base.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.base.subsystems.arcsystems.ARCSystemsContext;
import org.firstinspires.ftc.teamcode.base.subsystems.arcsystems.ConfigurableSubsystem;

import java.util.HashMap;
import java.util.List;

public class BertoBot {
    //base
    private final HardwareMap hwMap;
    private final JoinedTelemetry telemetry;
    public final GamepadEx driverOp;
    public final GamepadEx toolOp;

    private final List<LynxModule> lynxModules;

    //subsystem
    private final HashMap<String, SubsystemBase> subsystems;

    //localizer


    public BertoBot(HardwareMap hwMap, Telemetry ftcTelemetry, GamepadEx driverOp, GamepadEx toolOp) {
        this.hwMap = hwMap;
        this.telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), ftcTelemetry);
        this.driverOp = driverOp;
        this.toolOp = toolOp;

        this.lynxModules = hwMap.getAll(LynxModule.class);
        lynxModules.forEach((lynxModule -> {
            lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }));

        this.subsystems = new HashMap<>();
        this.initSubsystems();
    }

    private void initSubsystems() {
        //TODO: dynamic subsystem population (reflection?)
        ARCSystemsContext ctx = new ARCSystemsContext(
                hwMap,
                telemetry,
                driverOp,
                toolOp
        );

        this.subsystems.put("drivetrain", new Drivetrain(hwMap, this.telemetry));
        this.subsystems.put("intake", new Intake(hwMap, this.telemetry));
        this.subsystems.put("turret", new Shooter(hwMap, this.telemetry));


        this.subsystems.forEach(((s, subsystemBase) -> {
            if(subsystemBase instanceof ConfigurableSubsystem) {
                ConfigurableSubsystem configurable = (ConfigurableSubsystem) subsystemBase;
                CommandScheduler.getInstance().registerSubsystem(subsystemBase);
                configurable.configureBindings(ctx);

                Command defaultCmd = configurable.createDefaultCommand(ctx);
                if(defaultCmd != null && subsystemBase.getDefaultCommand() == null) {
                    subsystemBase.setDefaultCommand(defaultCmd);
                    telemetry.addLine("set data to cmd " + defaultCmd.toString());
                }


            }

            if(!(subsystemBase instanceof ConfigurableSubsystem)) {
                CommandScheduler.getInstance().registerSubsystem(subsystemBase);
            }

        }));


    }

    public void loop() {
        this.telemetry.update();
    }

    public HardwareMap getHwMap() {
        return hwMap;
    }

    public JoinedTelemetry getTelemetry() {
        return telemetry;
    }

    public SubsystemBase getSubsystem(String id) {
        return this.subsystems.get(id);
    }

    public HashMap<String, SubsystemBase> getSubsytems() {
        return this.subsystems;
    }
}
