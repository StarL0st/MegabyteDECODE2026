package org.firstinspires.ftc.teamcode.base.subsystems;

import com.bylazar.telemetry.JoinedTelemetry;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.base.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.base.subsystems.arcsystems.ARCSystemsContext;
import org.firstinspires.ftc.teamcode.base.subsystems.arcsystems.ConfigurableSubsystem;

/**
 * Intake subsystem
 * <p>
 * Only handles the intake motor control, has two states (INTAKE, TRANSFER).
 * Will be toggled by second driver control. `INTAKE` mode will only be allowed
 * to be used if the turret servo stop is in place.
 * `TRANSFER` will only be allowed if the same servo stop is open.
 */
public class Intake extends SubsystemBase implements ConfigurableSubsystem {
    private final JoinedTelemetry telemetry;
    public enum State {
        INTAKE,
        TRANSFER
    }
    public State state;

    private final Motor intakeMotor;
    private final ServoEx turretStopServo;

    private boolean stopState = false;

    private GamepadEx driverOp, toolOp;

    public Intake(HardwareMap hwMap, JoinedTelemetry telemetry) {
        this.telemetry = telemetry;
        this.intakeMotor = new Motor(hwMap, "intakeMotor");
        this.intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.intakeMotor.setRunMode(Motor.RunMode.RawPower);

        this.turretStopServo = new ServoEx(hwMap, "turretStopServo", 0, 50);
        this.turretStopServo.set(0);
        if(this.turretStopServo.get() == 0) this.stopState = true; //Fail-safe

        //Setup default state (INTAKE)
        if(stopState) this.state = State.INTAKE;
        if(!stopState) this.telemetry.addData("[INTAKE] WARNING: ", "FAILED TO CLOSE TURRET DOOR");
    }

    /**
     * Intake and transfer toggle logic.
     * <p>
     * If the current (and default state at init) is `State.INTAKE` then
     * the next state will be `State.TRANSFER`, but first checks are done
     * on the servo position to make sure it is at the right position.
     * <p>
     * In case the action is successful, it will be showed in the telemetry data in
     * the driver station with a short rumble on both controllers, indicating success,
     * same for failure, but with a longer rumble. The exception is if the closing of
     * the servo is successful, in which only the second driver will be notified by the
     * same short rumble, to not distract the main driver.
     */
    private void toggleState() {
        if(this.state.equals(State.INTAKE) && this.stopState) {
            this.turretStopServo.set(IntakeConstants.TURRET_STOP_OPEN);
            //success
            if(this.turretStopServo.get() == IntakeConstants.TURRET_STOP_OPEN) {
                this.stopState = false;
                this.state = State.TRANSFER;

                this.telemetry.addData("[TRANSFER]", "Opened turret servo stop");
                this.toolOp.gamepad.rumble(35);
                this.driverOp.gamepad.rumble(35);
            } else {
                this.telemetry.addData("[INTAKE] WARNING: ",
                        "FAILED TO OPEN TURRET STOP!");
                this.driverOp.gamepad.rumble(55);
                this.toolOp.gamepad.rumble(55);
            }
        } else if (this.state.equals(State.TRANSFER) && !this.stopState) {
            this.turretStopServo.set(IntakeConstants.TURRET_STOP_CLOSE);
            //success
            if(this.turretStopServo.get() == IntakeConstants.TURRET_STOP_CLOSE) {
                this.stopState = true;
                this.state = State.INTAKE;

                this.telemetry.addData("[TRANSFER]", "Closed turret servo stop");
                this.toolOp.gamepad.rumble(35);
            } else {
                this.telemetry.addData("[INTAKE] WARNING: ",
                        "FAILED TO CLOSE TURRET STOP!");
                this.driverOp.gamepad.rumble(55);
                this.toolOp.gamepad.rumble(55);
            }
        }
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
        this.driverOp = ctx.getDriverOp();
        this.toolOp = ctx.getToolOp();
        //First driver toggles intake for faster coordination
        ctx.getDriverOp().getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> {
                    if(this.state.equals(State.INTAKE)) {
                        if(this.intakeMotor.get() != IntakeConstants.INTAKE_MOTOR_POWER) {
                            this.intakeMotor.set(IntakeConstants.INTAKE_MOTOR_POWER);
                        }
                    } else if(this.state.equals(State.TRANSFER)) {
                        if(this.intakeMotor.get() != IntakeConstants.TRANSFER_MOTOR_POWER) {
                            this.intakeMotor.set(IntakeConstants.TRANSFER_MOTOR_POWER);
                        }
                    }
                })
                .whenReleased(() -> {
                   if(this.intakeMotor.get() != 0.0) {
                       this.intakeMotor.stopMotor();
                   }
                });

        //Second driver handles state changes
        ctx.getToolOp().getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenReleased(this::toggleState);
    }
}
