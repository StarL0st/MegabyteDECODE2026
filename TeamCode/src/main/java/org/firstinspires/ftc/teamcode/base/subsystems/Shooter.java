package org.firstinspires.ftc.teamcode.base.subsystems;

import com.bylazar.telemetry.JoinedTelemetry;
import com.pedropathing.control.PIDFController;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.base.subsystems.arcsystems.ARCSystemsContext;
import org.firstinspires.ftc.teamcode.base.subsystems.arcsystems.ConfigurableSubsystem;

public class Shooter extends SubsystemBase implements ConfigurableSubsystem {

    private final JoinedTelemetry telemetry;
    public enum State {
        READY, LAUNCH, OFF, RESET
    }

    private State state;
    private final Timer timer;

    private DcMotorEx flywheelMotor;
    //private final ServoEx turretRampServo;
    private final Limelight3A limelight;

    public PIDFController flywheelController;

    private boolean runFlywheel = false;
    private boolean runTurret = false;

    private final Vector robotToGoalVector = new Vector();
    private Vector launchVector = new Vector();


    public Shooter(HardwareMap hwMap, JoinedTelemetry telemetry) {
        this.timer = new Timer();
        this.telemetry = telemetry;

        this.flywheelMotor = hwMap.get(DcMotorEx.class, "turretFlywheelMotor");
        this.flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.flywheelController = new PIDFController(ShooterConstants.FLYWHEEL_PIDF);

        this.limelight = hwMap.get(Limelight3A.class, "limelight");
        this.setState(State.READY);
    }

    @Override
    public void periodic() {
        //state machine
        /*
        switch (state) {
            case READY:
                //ready logic
                break;

            case LAUNCH:
                //launch logic
                break;

            case OFF:
                //off logic
                break;

            case RESET:
                //reset everything
                break;
        }

         */

        if(state != State.OFF && state != State.RESET) {
            //goal targeting
            setFlywheelSpeed(1450);
        } else {
            this.flywheelMotor.setPower(ShooterConstants.FLYWHEEL_OFF);
            this.flywheelController.reset();
        }
    }

    public void setState(State s) {
        state = s;
        timer.resetTimer();
    }

    private void setFlywheelSpeed(double speed) {
        double flywheelError = speed - this.flywheelMotor.getVelocity();
        flywheelController.updateError(flywheelError);

        double flywheelPower = MathFunctions.clamp(flywheelController.run(), -1, 1);
        this.telemetry.addData("flywheel state", this.runFlywheel);
        this.telemetry.addData("flywheel speed", this.flywheelMotor.getVelocity());
        this.telemetry.addData("flywheel PID", flywheelPower);
        this.flywheelMotor.setPower(runFlywheel ? flywheelPower : 0);
    }

    @Override
    public Command createDefaultCommand(ARCSystemsContext ctx) {
        return null;
    }

    @Override
    public void configureBindings(ARCSystemsContext ctx) {
        ctx.getDriverOp().getGamepadButton(GamepadKeys.Button.TRIANGLE)
                .whenReleased(() -> {
                    this.runFlywheel = !this.runFlywheel;
                });
    }
}
