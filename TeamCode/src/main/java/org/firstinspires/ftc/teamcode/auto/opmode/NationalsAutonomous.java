package org.firstinspires.ftc.teamcode.auto.opmode;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.base.PoseTracker;
import org.firstinspires.ftc.teamcode.base.command.drivetrain.AcquireVisionCommand;
import org.firstinspires.ftc.teamcode.base.command.drivetrain.DriveToDistanceCommand;
import org.firstinspires.ftc.teamcode.base.command.drivetrain.PulsedFeedCommand;
import org.firstinspires.ftc.teamcode.base.command.drivetrain.StrafeCommand;
import org.firstinspires.ftc.teamcode.base.config.ConfigManager;
import org.firstinspires.ftc.teamcode.base.config.RobotConfig;
import org.firstinspires.ftc.teamcode.base.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.base.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.base.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.base.subsystems.IntakeAndTransfer;
import org.firstinspires.ftc.teamcode.base.subsystems.Shooter;

@Autonomous(name = "nationals auto", preselectTeleOp = "MasterTeleOp")
public class NationalsAutonomous extends CommandOpMode {

    private Drivetrain drivetrain;
    private Shooter shooter;
    private IntakeAndTransfer intake;

    private final JoinedTelemetry joinedTelemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

    private boolean scheduledAuto = false;
    private boolean isRedAlliance = true;
    private boolean isCloseAuto = true;

    SequentialCommandGroup closeSequence;
    SequentialCommandGroup farSequence;

    @Override
    public void initialize() {
        super.reset();

        RobotConfig config = ConfigManager.getConfig();
        if(config.getAlliance().equals(RobotConfig.Alliance.RED)) {
            this.isRedAlliance = true;
        } else if(config.getAlliance().equals(RobotConfig.Alliance.BLUE)) {
            this.isRedAlliance = false;
        }
        if(config.getStartPosition().equals(RobotConfig.StartPosition.CLOSE_SIDE)) {
            this.isCloseAuto = true;
        } else if(config.getStartPosition().equals(RobotConfig.StartPosition.FAR_SIDE)) {
            this.isCloseAuto = false;
        }

        telemetry.addData("[AUTO CONFIG]", "CONFIRM CONFIGURATION");
        telemetry.addData("[ALLIANCE]", config.getAlliance());
        telemetry.addData("[START POS]", config.getStartPosition());
        telemetry.update();

        drivetrain = new Drivetrain(hardwareMap, this.joinedTelemetry);
        shooter = new Shooter(hardwareMap, this.joinedTelemetry);
        intake = new IntakeAndTransfer(hardwareMap, this.joinedTelemetry);

        register(drivetrain, shooter, intake);
        //follower
        //follower = Constants.createFollower(hardwareMap);
        //follower.setStartingPose(new Pose(122, 122, Math.toRadians(45)));
        //buildPaths(follower);
        this.drivetrain.setAllMotors(0);
        this.shooter.setState(Shooter.State.OFF);
        this.shooter.setState(Shooter.State.READY);
        this.intake.setRunState(IntakeAndTransfer.RunState.OFF);
        this.intake.setState(IntakeAndTransfer.State.TRANSFER);

        closeSequence = new SequentialCommandGroup(
                new AcquireVisionCommand(
                        this.drivetrain,
                        DriveConstants.ACQUIRE_VISION_SPEED,
                        DriveConstants.ACQUIRE_VISION_TIMEOUT),
                new WaitCommand(300),
                new DriveToDistanceCommand(
                        this.drivetrain,
                        DriveConstants.TARGET_DIST),
                new InstantCommand(() -> {
                    this.shooter.setState(Shooter.State.LAUNCH);
                    telemetry.addData("[SHOOTER]", "READY");
                }),
                new WaitCommand(2000),
                new InstantCommand(),
                new InstantCommand(() -> {
                    this.shooter.setState(Shooter.State.OFF);
                    telemetry.addData("[SHOOTER]", "OFF");
                }),
                new WaitCommand(500),
                new StrafeCommand(
                        this.drivetrain,
                        isRedAlliance ? -DriveConstants.STRAFE_SPEED : DriveConstants.STRAFE_SPEED,
                        0,
                        DriveConstants.STRAFE_DURATION_MS),
                new WaitCommand(300),
                new StrafeCommand(
                        this.drivetrain,
                        0,
                        DriveConstants.FORWARD_STRAFE_SPEED,
                        DriveConstants.FORWARD_STRAFE_DURATION_MS),
                new InstantCommand(() -> telemetry.addData("[AUTO]", "Complete!"))
        );

        farSequence = new SequentialCommandGroup(
                new StrafeCommand(
                        this.drivetrain,
                        isRedAlliance ? -0.6 : 0.6,
                        0,
                        750
                )
        );
    }

    @Override
    public void run() {
        super.run();
        if(!scheduledAuto) {
            if(isCloseAuto) {
                schedule(closeSequence);
            } else {
                schedule(farSequence);
            }
            this.scheduledAuto = true;
            telemetry.addData("cmd sequence", "registered cmd sequence");
        }
        telemetry.addData("dist data", PoseTracker.distRegression(PoseTracker.INSTANCE.getTa()));

        telemetry.update();
    }
}
