package org.firstinspires.ftc.teamcode.base.subsystems;

import com.bylazar.telemetry.JoinedTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.base.command.drivetrain.DriveCommand;
import org.firstinspires.ftc.teamcode.base.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.base.subsystems.arcsystems.ARCSystemsContext;
import org.firstinspires.ftc.teamcode.base.subsystems.arcsystems.ConfigurableSubsystem;

public class Drivetrain extends SubsystemBase implements ConfigurableSubsystem {
    private final JoinedTelemetry telemetry;

    private final Motor left_back_drive;
    private final Motor left_front_drive;
    private final Motor right_back_drive;
    private final Motor right_front_drive;

    private final IMU imu;

    private boolean runAutoTurn;
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
    }

    public void driveWithController(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        double y = -forwardSpeed;
        double x = -strafeSpeed;
        double rx = -turnSpeed;

        if(runAutoTurn) {
            double error = this.headingSetpoint - this.getHeading(AngleUnit.DEGREES);

            if(error > 180) {
                error -= 360;
            } else if(error < -180) {
                error += 360;
            }

            rx = DriveConstants.TURN_P_GAIN * error;
            rx = Math.min(Math.max(rx, -0.4), 0.4);
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        left_front_drive.set(frontLeftPower);
        left_back_drive.set(backLeftPower);
        right_front_drive.set(frontRightPower);
        right_back_drive.set(backRightPower);
    }

    @Override
    public void periodic() {
        telemetry.addData("Robot Heading", this.getHeading(AngleUnit.DEGREES));
    }

    public double getHeading(AngleUnit unit) {
        return this.imu.getRobotYawPitchRollAngles().getYaw(unit);
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
        ctx.getDriverOp().getGamepadButton(GamepadKeys.Button.CIRCLE)
                .whenPressed(() -> {
                    this.runAutoTurn = true;
                    this.headingSetpoint = 270;
                })
                .whenReleased(() -> {
                    this.runAutoTurn = false;
                });

        ctx.getDriverOp().getGamepadButton(GamepadKeys.Button.CROSS)
                .whenPressed(() -> {
                    this.runAutoTurn = true;
                    this.headingSetpoint = 45;
                })
                .whenReleased(() -> {
                    this.runAutoTurn = false;
                });
    }
}
