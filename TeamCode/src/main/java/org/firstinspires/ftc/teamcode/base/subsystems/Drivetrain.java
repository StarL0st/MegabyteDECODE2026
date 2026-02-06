package org.firstinspires.ftc.teamcode.base.subsystems;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.base.PoseTracker;
import org.firstinspires.ftc.teamcode.base.command.drivetrain.DriveCommand;
import org.firstinspires.ftc.teamcode.base.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.base.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.base.subsystems.arcsystems.ARCSystemsContext;
import org.firstinspires.ftc.teamcode.base.subsystems.arcsystems.ConfigurableSubsystem;

/**
 * Drivetrain subsystem
 * <p>
 * Handles ALL robot movement logic, utilizing a field-centric drive system.
 */
public class Drivetrain extends SubsystemBase implements ConfigurableSubsystem {
    private final JoinedTelemetry telemetry;

    private final Motor left_back_drive;
    private final Motor left_front_drive;
    private final Motor right_back_drive;
    private final Motor right_front_drive;

    private final IMU imu;

    private boolean runAutoTurn;

    // ---- AIMING ----
    private boolean runAimLock;
    private double error = 0;
    private double lastError = 0;
    private double goalX = 0;
    private double curTime = 0;
    private double lastTime = 0;

    // ---- AUTO-DRIVE ----
    private boolean runDistanceControl = false;
    private double targetDistance = 0;
    private double distanceError = 0;
    private double lastDistanceError = 0;
    private double distanceLastTime = 0;

    private Timer timer;

    private double headingSetpoint;

    public Drivetrain(HardwareMap hardwareMap, JoinedTelemetry telemetry) {
        //super();
        this.telemetry = telemetry;

        this.left_back_drive = new Motor(hardwareMap, "backLeftMotor");
        this.left_front_drive = new Motor(hardwareMap, "frontLeftMotor");
        this.right_back_drive = new Motor(hardwareMap, "backRightMotor");
        this.right_front_drive = new Motor(hardwareMap, "frontRightMotor");

        this.left_back_drive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.left_front_drive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.right_back_drive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.right_front_drive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        this.right_back_drive.setInverted(true);
        this.right_front_drive.setInverted(true);

        this.imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot revOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        this.imu.initialize(new IMU.Parameters(revOrientation));
        this.timer = new Timer();
        this.timer.resetTimer();
        this.curTime = timer.getElapsedTime();
    }

    /**
     * Autonomous PID-controlled drive to a target distance based on vision
     * Call this repeatedly in a loop until it returns true
     * @return true if at target distance, false otherwise
     */
    public boolean driveToDistance() {
        if(!runDistanceControl) return true;

        double currentDistance = PoseTracker.distRegression(PoseTracker.INSTANCE.getTa());
        distanceError = targetDistance - currentDistance;

        // If within tolerance, stop and return true
        if(Math.abs(distanceError) < DriveConstants.DISTANCE_TOLERANCE) {
            this.setAllMotors(0);
            return true;
        }

        double pTerm = distanceError * DriveConstants.DISTANCE_P_GAIN;

        curTime = timer.getElapsedTime();
        double dT = curTime - distanceLastTime;
        double dTerm = ((distanceError - lastDistanceError) / dT) * DriveConstants.DISTANCE_D_GAIN;

        double forwardPower = Range.clip(pTerm + dTerm, -0.5, 0.5);

        // Drive straight forward/backward (no strafe, no turn)
        double denominator = Math.max(Math.abs(forwardPower), 1);
        double frontLeftPower = forwardPower / denominator;
        double backLeftPower = forwardPower / denominator;
        double frontRightPower = forwardPower / denominator;
        double backRightPower = forwardPower / denominator;

        left_front_drive.set(frontLeftPower);
        left_back_drive.set(backLeftPower);
        right_front_drive.set(frontRightPower);
        right_back_drive.set(backRightPower);

        lastDistanceError = distanceError;
        distanceLastTime = curTime;

        return false;
    }

    /**
     * Start autonomous distance control
     * @param distance target distance in inches
     */
    public void setTargetDistance(double distance) {
        this.targetDistance = distance;
        this.runDistanceControl = true;
        this.distanceLastTime = timer.getElapsedTime();
        this.lastDistanceError = 0;
    }

    /**
     * Stop distance control
     */
    public void stopDistanceControl() {
        this.runDistanceControl = false;
        this.setAllMotors(0);
    }

    /**
     * Check if distance control is finished
     * @return true if at target or not running
     */
    public boolean isAtTarget() {
        if(!runDistanceControl) return true;
        double currentDistance = PoseTracker.distRegression(PoseTracker.INSTANCE.getTa());
        return Math.abs(targetDistance - currentDistance) < DriveConstants.DISTANCE_TOLERANCE;
    }

    /**
     * Set individual motor powers (for manual control in auto)
     */
    public void setMotorPowers(double frontLeft, double backLeft, double frontRight, double backRight) {
        left_front_drive.set(frontLeft);
        left_back_drive.set(backLeft);
        right_front_drive.set(frontRight);
        right_back_drive.set(backRight);
    }

    public void setAllMotors(double power) {
        if(power == 0) {
            left_front_drive.stopMotor();
            left_back_drive.stopMotor();
            right_front_drive.stopMotor();
            right_back_drive.stopMotor();
        }
        left_front_drive.set(power);
        left_back_drive.set(power);
        right_front_drive.set(power);
        right_back_drive.set(power);
    }

    /**
     * Run by `DriveCommand`, handles TeleOp movement and angle setpoints.
     * @param strafeSpeed left joystick X value
     * @param forwardSpeed left joystick Y value
     * @param turnSpeed right joystick X value
     */
    public void driveWithController(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        double y = -forwardSpeed;
        double x = -strafeSpeed * 1.1;
        double rx = -turnSpeed;

        if(runAutoTurn) {
            rx = this.autoTurn(rx);
        }

        if(runAimLock) {
            rx = this.goalAim(rx);
            telemetry.addData("rx data (auto aim)", rx);
        } else {
            lastTime = timer.getElapsedTime();
            lastError = 0;
        }

        //double botHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        //double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        //double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        //rotX = rotX * 1.1;

        //double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        //double frontLeftPower = (rotY + rotX + rx) / denominator;
        //double backLeftPower = (rotY - rotX + rx) / denominator;
        //double frontRightPower = (rotY - rotX - rx) / denominator;
        //double backRightPower = (rotY + rotX - rx) / denominator;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        left_front_drive.set(frontLeftPower);
        left_back_drive.set(backLeftPower);
        right_front_drive.set(frontRightPower);
        right_back_drive.set(backRightPower);
    }

    public double autoTurn(double rx) {
        double error = this.headingSetpoint - PoseTracker.INSTANCE.getHeading();

        if(error > 180) {
            error -= 360;
        } else if(error < -180) {
            error += 360;
        }

        rx = DriveConstants.TURN_P_GAIN * error;
        rx = Math.min(Math.max(rx, -0.4), 0.4);
        return rx;
    }

    public double goalAim(double rx) {
        error = goalX - PoseTracker.INSTANCE.tX;

        if(Math.abs(error) < DriveConstants.ANGLE_TOLERANCE) {
            rx = 0;
        } else {
            double pTerm = error * DriveConstants.AIM_P_GAIN;

            curTime = timer.getElapsedTime();
            double dT = curTime - lastTime;
            double dTerm = ((error - lastError) / dT) * DriveConstants.AIM_D_GAIN;

            rx = Range.clip(pTerm + dTerm, -0.4, 0.4);

            lastError = error;
            lastTime = curTime;
        }

        return rx;
    }

    public void setTargetHeading(double angle) {
        this.headingSetpoint = angle;
    }

    @Override
    public void periodic() {
        telemetry.addData("Robot Heading", this.getHeading(AngleUnit.DEGREES));
        telemetry.addData("running auto turn", this.runAutoTurn);
        telemetry.addData("running aim lock", this.runAimLock);
        telemetry.addData("going to angle", this.headingSetpoint);
        telemetry.addData("run to distance", this.runDistanceControl);
        telemetry.addData("target dist", this.targetDistance);
        PoseTracker.INSTANCE.setHeading(this.getHeading(AngleUnit.DEGREES));
    }

    public double getHeading(AngleUnit unit) {
        return -this.imu.getRobotYawPitchRollAngles().getYaw(unit);
    }

    public void resetIMU() {
        this.imu.resetYaw();
    }

    @Override
    public Command createDefaultCommand(ARCSystemsContext ctx) {
        return new DriveCommand(this,
                ctx.getDriverOp()::getLeftX,
                ctx.getDriverOp()::getLeftY,
                ctx.getDriverOp()::getRightX);
    }

    @Override
    public void configureBindings(ARCSystemsContext ctx) {
        ctx.getDriverOp().getGamepadButton(GamepadKeys.Button.OPTIONS)
                .whenReleased(new InstantCommand(
                        this::resetIMU,
                        this
                ));
        ctx.getDriverOp().getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> {
                    this.runAimLock = true;
                })
                .whenReleased(() -> {
                    this.runAimLock = false;
                });
    }
}
