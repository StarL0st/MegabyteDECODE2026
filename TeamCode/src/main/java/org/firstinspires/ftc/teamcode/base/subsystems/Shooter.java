package org.firstinspires.ftc.teamcode.base.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.control.PIDFController;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
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
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.base.PoseTracker;
import org.firstinspires.ftc.teamcode.base.config.ConfigManager;
import org.firstinspires.ftc.teamcode.base.config.RobotConfig;
import org.firstinspires.ftc.teamcode.base.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.base.subsystems.arcsystems.ARCSystemsContext;
import org.firstinspires.ftc.teamcode.base.subsystems.arcsystems.ConfigurableSubsystem;

@Configurable
public class Shooter extends SubsystemBase implements ConfigurableSubsystem {

    private final JoinedTelemetry telemetry;
    public enum State {
        READY, LAUNCH, OFF, RESET
    }

    private State state;
    private final Timer timer;

    private final Motor flywheelMotor;
    private final ServoEx turretRampServo;
    private final Limelight3A limelight;

    public PIDFController flywheelController;

    private boolean runFlywheel = false;
    private boolean runFlywheelStatic = false;

    private final Vector robotToGoalVector = new Vector();
    private Vector launchVector = new Vector();

    public Shooter(HardwareMap hwMap, JoinedTelemetry telemetry) {
        this.timer = new Timer();
        this.telemetry = telemetry;

        this.flywheelMotor = new Motor(hwMap, "turretFlywheelMotor");
        this.flywheelMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.flywheelMotor.setRunMode(Motor.RunMode.VelocityControl);
        this.flywheelMotor.setVeloCoefficients(
                ShooterConstants.flywheelKp,
                ShooterConstants.flywheelKi,
                ShooterConstants.flywheelKd);
        this.flywheelMotor.setFeedforwardCoefficients(0, ShooterConstants.flywheelKf);

        this.turretRampServo = new ServoEx(hwMap, "turretRampServo", 0, 50);
        this.turretRampServo.setInverted(true);
        //this.turretRampServo.getServo().setPosition(15);

        this.limelight = hwMap.get(Limelight3A.class, "limelight");
        this.limelight.setPollRateHz(100);

        this.limelight.start();
        RobotConfig config = ConfigManager.getConfig();
        if(config.getAlliance().equals(RobotConfig.Alliance.BLUE)) {
            this.limelight.pipelineSwitch(0);
        } else if(config.getAlliance().equals(RobotConfig.Alliance.RED)) {
            this.limelight.pipelineSwitch(1);
        }
        this.setState(State.READY);
        timer.resetTimer();
    }

    @Override
    public void periodic() {
        telemetry.addLine("  ---- SHOOTER ----  ");
        telemetry.addData("[STATE]", this.state.toString());
        //state machine
        switch (state) {
            case READY:
                //ready logic
                this.setHoodAngle(ShooterConstants.servoTargetPos);
                break;

            case LAUNCH:
                //launch logic
                this.runFlywheel = true;
                break;

            case OFF:
                //off logic
                this.runFlywheel = false;
                break;

            case RESET:
                //reset everything
                break;
        }



        if(state != State.OFF && state != State.RESET) {
            //goal targeting

            double flywheelRegression = flywheelRegression(PoseTracker.distRegression(PoseTracker.INSTANCE.getTa()));
            setFlywheelSpeed(flywheelRegression);
            telemetry.addData("[FLYWHEEL POWER]", flywheelRegression);
            double hoodRegression = hoodRegression(PoseTracker.distRegression(PoseTracker.INSTANCE.getTa()));
            //setHoodAngle(hoodRegression);
            //telemetry.addData("servo regression", hoodRegression);
            //telemetry.addData("debug flywheel target speed", ShooterConstants.targetFlywheelSpeed);
            updateLimelight();

        } else {
            if(this.flywheelMotor.get() != 0) {
                this.flywheelMotor.stopMotor();
            }
            this.setHoodAngle(ShooterConstants.servoTargetPos);
        }

    }

    public double hoodRegression(double x) {
        double y = -1.69426e-7 * Math.pow(x, 4)
                + 0.0000936065 * Math.pow(x, 3)
                - 0.0176738 * Math.pow(x, 2)
                + 1.38363 * x
                - 32.41501;

        return Math.max(0.0, Math.min(1.0, y));
    }


    public double flywheelRegression(double x) {
        double y = 3.97033e-9 * Math.pow(x, 3)
                - 0.00000343732 * Math.pow(x, 2)
                + 0.00169896 * x
                + 0.36;

        return Math.max(0.0, Math.min(1.0, y));
    }

    public void setState(State s) {
        state = s;
        timer.resetTimer();
    }

    private void updateLimelight() {
        this.limelight.updateRobotOrientation(PoseTracker.INSTANCE.getNormalizedHeading());
        //telemetry.addData("normalized heading", PoseTracker.INSTANCE.getNormalizedHeading());
        telemetry.addLine("   ---- LIMELIGHT ----   ");
        LLResult currentResult = this.limelight.getLatestResult();
        if(currentResult != null && currentResult.isValid()) {
            double staleness = currentResult.getStaleness();
            if(staleness < 100) {
                telemetry.addData("[LL3A Data Last Update]", staleness + "ms");
            }  else {
                telemetry.addData("[LL3A Data]", "Data is Old! " + staleness + "ms");
            }
            PoseTracker.INSTANCE.setTa(currentResult.getTa());
            telemetry.addData("[LL TX]", PoseTracker.INSTANCE.getTx());
            PoseTracker.INSTANCE.setTx(currentResult.getTx());
            telemetry.addData("[LL TA]", PoseTracker.INSTANCE.getTa());
            Pose3D botPose = currentResult.getBotpose_MT2();
            if(botPose != null) {
                PoseTracker.INSTANCE.setLLPose(botPose);
                //telemetry.addData("pose", botPose.getPosition().toString());
                //telemetry.addData("orientation pose", botPose.getOrientation().toString());
            }
            telemetry.addData("[LL TAG DISTANCE (CM)]", PoseTracker.distRegression(
                    PoseTracker.INSTANCE.getTa()
            ));
        }
    }

    private void setFlywheelSpeed(double speed) {
        double flywheelError = speed - this.flywheelMotor.getCorrectedVelocity();
        telemetry.addLine("  ---- FLYWHEEL ----  ");
        this.telemetry.addData("[RUNNING?]", this.runFlywheel);
        this.telemetry.addData("[SPEED (RPM)]", this.flywheelMotor.getCorrectedVelocity());
        this.telemetry.addData("[POWER]", speed);
        if(this.runFlywheel) {
            this.flywheelMotor.set(speed);
        } else {
            this.flywheelMotor.stopMotor();
        }
        if(this.runFlywheelStatic) {
            this.flywheelMotor.set(ShooterConstants.FLYWHEEL_STATIC_SPEED);
        } else {
            this.flywheelMotor.stopMotor();
        }
    }

    private void setHoodAngle(double angle) {
        if(angle < ShooterConstants.HOOD_LOW || angle > ShooterConstants.HOOD_HIGH) return;
        if(angle == this.turretRampServo.get()) return;
        this.turretRampServo.set(angle);
    }

    @Override
    public Command createDefaultCommand(ARCSystemsContext ctx) {
        return null;
    }

    @Override
    public void configureBindings(ARCSystemsContext ctx) {
        ctx.getToolOp().getGamepadButton(GamepadKeys.Button.TRIANGLE)
                .whenReleased(() -> {
                    this.runFlywheel = !this.runFlywheel;
                });
        ctx.getToolOp().getGamepadButton(GamepadKeys.Button.A)
                        .whenReleased(() -> {
                            this.runFlywheelStatic = !this.runFlywheelStatic;
                        });
        ctx.getToolOp().getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(() -> {
                    this.setHoodAngle(ShooterConstants.servoTargetPos);
                });

        ctx.getToolOp().getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(() -> {
                    this.limelight.captureSnapshot("test field view");
                });
    }
}
