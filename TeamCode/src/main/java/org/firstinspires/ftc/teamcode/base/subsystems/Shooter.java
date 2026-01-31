package org.firstinspires.ftc.teamcode.base.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
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

    private final Vector robotToGoalVector = new Vector();
    private Vector launchVector = new Vector();

    public static double targetFlywheelSpeed = 0.8;

    //flywheel
    private double flywheelKp = 20;
    private double flywheelKi = 0;
    private double flywheelKd = 0;
    private double flywheelKf = 0.7;

    public Shooter(HardwareMap hwMap, JoinedTelemetry telemetry) {
        this.timer = new Timer();
        this.telemetry = telemetry;

        this.flywheelMotor = new Motor(hwMap, "turretFlywheelMotor");
        this.flywheelMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.flywheelMotor.setRunMode(Motor.RunMode.VelocityControl);
        this.flywheelMotor.setVeloCoefficients(flywheelKp, flywheelKi, flywheelKd);
        this.flywheelMotor.setFeedforwardCoefficients(0, flywheelKf);

        this.turretRampServo = new ServoEx(hwMap, "turretRampServo", 0, 50);
        this.turretRampServo.setInverted(true);
        //this.turretRampServo.getServo().setPosition(0);
        //this.flywheelController = new PIDFController(ShooterConstants.FLYWHEEL_PIDF);

        this.limelight = hwMap.get(Limelight3A.class, "limelight");
        this.limelight.setPollRateHz(100);

        this.limelight.start();
        this.limelight.pipelineSwitch(1);
        this.setState(State.READY);


    }

    @Override
    public void periodic() {
        //state machine

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



        if(state != State.OFF && state != State.RESET) {
            //goal targeting
            setFlywheelSpeed(targetFlywheelSpeed);
            updateLimelight();

        } else {
            this.flywheelMotor.set(ShooterConstants.FLYWHEEL_OFF);
            this.setHoodAngle(ShooterConstants.HOOD_LOW);
            this.flywheelController.reset();
        }
    }

    public void setState(State s) {
        state = s;
        timer.resetTimer();
    }

    private void updateLimelight() {
        this.limelight.updateRobotOrientation(PoseTracker.INSTANCE.getNormalizedHeading());
        LLResult currentResult = this.limelight.getLatestResult();
        if(currentResult != null && currentResult.isValid()) {
            double staleness = currentResult.getStaleness();
            if(staleness < 100) {
                telemetry.addData("LL3A Data Last Update", staleness + "ms");
            }  else {
                telemetry.addData("LL3A Data", "Data is Old! " + staleness + "ms");
            }
            PoseTracker.INSTANCE.setTa(currentResult.getTa());
            telemetry.addData("tx", PoseTracker.INSTANCE.getTx());
            PoseTracker.INSTANCE.setTx(currentResult.getTx());
            telemetry.addData("ta", PoseTracker.INSTANCE.getTa());
            Pose3D botPose = currentResult.getBotpose_MT2();
            if(botPose != null) {
                telemetry.addData("pose", botPose.getPosition().toString());
                telemetry.addData("orientation pose", botPose.getOrientation().toString());
            }
            telemetry.addData("regression dist", PoseTracker.distRegression(
                    PoseTracker.INSTANCE.getTa()
            ));
        }
    }

    private void setFlywheelSpeed(double speed) {
        double flywheelError = speed - this.flywheelMotor.getCorrectedVelocity();
        this.telemetry.addData("flywheel state", this.runFlywheel);
        this.telemetry.addData("flywheel speed", this.flywheelMotor.getCorrectedVelocity());
        this.telemetry.addData("flywheel power", speed);
        this.flywheelMotor.set(runFlywheel ? speed : 0);
    }

    private void setHoodAngle(double angle) {
        if(angle < 0 || angle > 45) return;
        this.turretRampServo.set(angle);
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
        ctx.getDriverOp().getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenReleased(() -> {
                    this.setHoodAngle(45);
                });
        ctx.getDriverOp().getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenReleased(() -> {
                    this.setHoodAngle(0);
                });
    }
}
