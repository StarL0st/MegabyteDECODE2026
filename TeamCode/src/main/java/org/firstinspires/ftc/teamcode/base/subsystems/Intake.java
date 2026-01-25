package org.firstinspires.ftc.teamcode.base.subsystems;

import com.bylazar.telemetry.JoinedTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.base.subsystems.arcsystems.ARCSystemsContext;
import org.firstinspires.ftc.teamcode.base.subsystems.arcsystems.ConfigurableSubsystem;

public class Intake extends SubsystemBase implements ConfigurableSubsystem {
    private final Motor intakeMotor;
    private final ServoEx transferServo;

    public Intake(HardwareMap hwMap, JoinedTelemetry telemetry) {
        this.intakeMotor = new Motor(hwMap, "intakeMotor");
        this.intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.intakeMotor.setRunMode(Motor.RunMode.RawPower);

        this.transferServo = new ServoEx(hwMap, "transferServo", 0, 30);
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

        ctx.getDriverOp().getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> {
                    if(this.transferServo.get() != 20) {
                        this.transferServo.set(0);
                    }
                })
                .whenReleased(() -> {
                    if(this.transferServo.get() != 0) {
                        this.transferServo.set(0);
                    }
                });
    }


}
