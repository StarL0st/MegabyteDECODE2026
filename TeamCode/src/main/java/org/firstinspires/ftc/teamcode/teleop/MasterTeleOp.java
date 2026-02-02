package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.base.BertoBot;
import org.firstinspires.ftc.teamcode.base.command.drivetrain.DriveCommand;
import org.firstinspires.ftc.teamcode.base.subsystems.Drivetrain;
/**
Master TeleOp

To be used during competition, avoid making changes here as it is exclusively
to register the robot logic against the Control Hub, so the OpMode shows up.
 */
@TeleOp(name = "MasterTeleOp", group = "FTC MEGABYTE NATIONALS")
public class MasterTeleOp extends CommandOpMode {
    BertoBot bertoBot;
    @Override
    public void initialize() {
        this.bertoBot = new BertoBot(
                hardwareMap,
                telemetry,
                new GamepadEx(gamepad1),
                new GamepadEx(gamepad2)
        );
        schedule(new RunCommand(this.bertoBot::loop));
    }
}
