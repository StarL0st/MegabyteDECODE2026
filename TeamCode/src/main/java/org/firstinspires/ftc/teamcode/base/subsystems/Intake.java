package org.firstinspires.ftc.teamcode.base.subsystems;

import com.bylazar.telemetry.JoinedTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.base.subsystems.arcsystems.ARCSystemsContext;
import org.firstinspires.ftc.teamcode.base.subsystems.arcsystems.ConfigurableSubsystem;

public class Intake extends SubsystemBase implements ConfigurableSubsystem {
    private final Motor intakeMotor;

    public Intake(HardwareMap hwMap, JoinedTelemetry telemetry) {
        this.intakeMotor = new Motor(hwMap, "intakeMotor");
        this.intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.intakeMotor.setRunMode(Motor.RunMode.RawPower);
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    @Override
    public Command createDefaultCommand(ARCSystemsContext ctx) {
        return null;
    }

    @Override
    public void configureBindings(ARCSystemsContext ctx) {
        ctx.getDriverOp().getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> {
                    if(this.intakeMotor.get() != 0.9) {
                        this.intakeMotor.set(0.9);
                    }
                })
                .whenReleased(() -> {
                   if(this.intakeMotor.get() != 0.0) {
                       this.intakeMotor.stopMotor();
                   }
                });
    }


}
