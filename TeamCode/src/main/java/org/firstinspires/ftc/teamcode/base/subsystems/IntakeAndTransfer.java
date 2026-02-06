package org.firstinspires.ftc.teamcode.base.subsystems;

import com.bylazar.telemetry.JoinedTelemetry;
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
 * IntakeAndTransfer subsystem
 * <p>
 * Only handles the intake motor control, has two states (INTAKE, TRANSFER).
 * Will be toggled by second driver control. `INTAKE` mode will only be allowed
 * to be used if the turret servo stop is in place.
 * `TRANSFER` will only be allowed if the same servo stop is open.
 */
public class IntakeAndTransfer extends SubsystemBase implements ConfigurableSubsystem {
    private final JoinedTelemetry telemetry;
    public enum State {
        INTAKE,
        TRANSFER
    }

    public enum RunState {
        RUNNING,
        REVERSE,
        OFF
    }

    public State state;
    public RunState runState;

    private final Motor intakeMotor;
    private final ServoEx turretStopServo;

    private boolean stopState = false;

    private GamepadEx driverOp, toolOp;

    public IntakeAndTransfer(HardwareMap hwMap, JoinedTelemetry telemetry) {
        this.telemetry = telemetry;
        this.intakeMotor = new Motor(hwMap, "intakeMotor");
        this.intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.intakeMotor.setRunMode(Motor.RunMode.RawPower);
        this.intakeMotor.setInverted(true);

        this.turretStopServo = new ServoEx(hwMap, "turretStopServo", 0, 50);
        this.turretStopServo.set(0);
        if(this.turretStopServo.get() == 0) this.stopState = true; //Fail-safe

        //Setup default state (INTAKE)
        this.state = State.INTAKE;
        if(!stopState) this.telemetry.addData("[INTAKE] WARNING: ", "FAILED TO CLOSE TURRET DOOR");
        this.runState = RunState.OFF;
    }

    /**
     * IntakeAndTransfer and transfer toggle logic.
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
    public void setRunState(RunState state) {
        this.runState = state;
    }

    public void setState(State state) {
        this.state = state;
    }

    /**
     * Set motor speeds directly (for pulsed feeding)
     * @param transferSpeed transfer motor speed (-1.0 to 1.0)
     */
    public void setMotorSpeed(double transferSpeed) {
        intakeMotor.set(transferSpeed);
    }

    public void stop() {
        this.intakeMotor.stopMotor();
    }

    @Override
    public void periodic() {
        super.periodic();
        telemetry.addData("[INTAKE & TRANSFER]", "Current State: " + this.state.toString());
        telemetry.addData("[INTAKE & TRANSFER]", "Current Run State: " + this.runState.toString());


        switch (runState) {
            case OFF:
                if(this.intakeMotor.get() != 0.0) {
                    this.intakeMotor.stopMotor();
                }
                break;

            case RUNNING:
                if(this.state.equals(State.INTAKE)) {
                    if(this.intakeMotor.get() != IntakeConstants.INTAKE_MOTOR_POWER) {
                        this.intakeMotor.set(IntakeConstants.INTAKE_MOTOR_POWER);
                    }
                } else if(this.state.equals(State.TRANSFER)) {
                    if(this.intakeMotor.get() != IntakeConstants.TRANSFER_MOTOR_POWER) {
                        this.intakeMotor.set(IntakeConstants.TRANSFER_MOTOR_POWER);
                    }
                }
                break;

            case REVERSE:
                if(this.intakeMotor.get() != IntakeConstants.TRANSFER_REVERSE_POWER) {
                    this.intakeMotor.set(IntakeConstants.TRANSFER_REVERSE_POWER);
                }
                break;
        }
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
                    if(this.runState != RunState.RUNNING) {
                        this.runState = RunState.RUNNING;
                    }
                })
                .whenReleased(() -> {
                    if(this.runState != RunState.OFF) {
                        this.runState = RunState.OFF;
                    }
                });
        ctx.getToolOp().getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> {
                    if(this.runState != RunState.RUNNING) {
                        this.runState = RunState.RUNNING;
                    }
                })
                .whenReleased(() -> {
                    if(this.runState != RunState.OFF) {
                        this.runState = RunState.OFF;
                    }
                });


        //Second driver handles state changes
        ctx.getToolOp().getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenReleased(this::toggleState);

        ctx.getToolOp().getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> {
                    if(this.runState != RunState.REVERSE) {
                        this.runState = RunState.REVERSE;
                    }
                })
                .whenReleased(() -> {
                    if(this.runState != RunState.OFF) {
                        this.runState = RunState.OFF;
                    }
                });
        ctx.getDriverOp().getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> {
                    if(this.runState != RunState.REVERSE) {
                        this.runState = RunState.REVERSE;
                    }
                })
                .whenReleased(() -> {
                    if(this.runState != RunState.OFF) {
                        this.runState = RunState.OFF;
                    }
                });
    }
}
