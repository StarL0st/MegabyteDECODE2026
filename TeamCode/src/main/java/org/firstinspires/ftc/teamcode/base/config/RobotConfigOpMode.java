package org.firstinspires.ftc.teamcode.base.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.base.config.ConfigManager;
import org.firstinspires.ftc.teamcode.base.config.RobotConfig;

@TeleOp(name = "Robot Configuration", group = "Config")
public class RobotConfigOpMode extends OpMode {

    private RobotConfig.Alliance selectedAlliance;
    private RobotConfig.StartPosition selectedPosition;

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastA = false;
    private boolean lastB = false;

    private enum ConfigState {
        SELECTING,
        CONFIRMING
    }

    private ConfigState state = ConfigState.SELECTING;

    @Override
    public void init() {
        // Load current config
        RobotConfig current = ConfigManager.getConfig();
        selectedAlliance = current.getAlliance();
        selectedPosition = current.getStartPosition();

        telemetry.addData("Status", "Ready to Configure");
        telemetry.addData("", "");
        telemetry.addData("Controls", "");
        telemetry.addData("D-Pad Up/Down", "Change Alliance");
        telemetry.addData("D-Pad Left/Right", "Change Position");
        telemetry.addData("A Button", "Confirm & Save");
        telemetry.addData("B Button", "Cancel");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Handle input based on state
        if (state == ConfigState.SELECTING) {
            handleSelection();
            displaySelection();
        } else if (state == ConfigState.CONFIRMING) {
            handleConfirmation();
            displayConfirmation();
        }
    }

    private void handleSelection() {
        // Toggle alliance with D-pad up/down
        if (gamepad1.dpad_up && !lastDpadUp) {
            selectedAlliance = RobotConfig.Alliance.RED;
        }
        if (gamepad1.dpad_down && !lastDpadDown) {
            selectedAlliance = RobotConfig.Alliance.BLUE;
        }

        // Toggle position with D-pad left/right
        if (gamepad1.dpad_left && !lastDpadLeft) {
            selectedPosition = RobotConfig.StartPosition.FAR_SIDE;
        }
        if (gamepad1.dpad_right && !lastDpadRight) {
            selectedPosition = RobotConfig.StartPosition.CLOSE_SIDE;
        }

        // Enter confirmation state
        if (gamepad1.a && !lastA) {
            state = ConfigState.CONFIRMING;
        }

        updateButtonStates();
    }

    private void handleConfirmation() {
        // Confirm and save
        if (gamepad1.a && !lastA) {
            RobotConfig newConfig = new RobotConfig(selectedAlliance, selectedPosition);
            ConfigManager.setConfig(newConfig);

            telemetry.clear();
            telemetry.addData("Status", "✓ SAVED!");
            telemetry.addData("", "");
            telemetry.addData("Configuration", newConfig.toString());
            telemetry.addData("", "");
            telemetry.addData("", "You can now close this OpMode");
            telemetry.update();

            requestOpModeStop();
        }

        // Cancel and go back
        if (gamepad1.b && !lastB) {
            state = ConfigState.SELECTING;
        }

        updateButtonStates();
    }

    private void displaySelection() {
        telemetry.clear();
        telemetry.addData("=== ROBOT CONFIGURATION ===", "");
        telemetry.addData("", "");

        // Display alliance selection
        String allianceDisplay = selectedAlliance == RobotConfig.Alliance.RED
                ? ">>> RED <<<" : "    RED";
        telemetry.addData("Alliance (D-Pad Up)", allianceDisplay);

        allianceDisplay = selectedAlliance == RobotConfig.Alliance.BLUE
                ? ">>> BLUE <<<" : "    BLUE";
        telemetry.addData("         (D-Pad Down)", allianceDisplay);

        telemetry.addData("", "");

        // Display position selection
        String positionDisplay = selectedPosition == RobotConfig.StartPosition.FAR_SIDE
                ? ">>> FAR SIDE <<<" : "    FAR SIDE";
        telemetry.addData("Start Pos (D-Pad Left)", positionDisplay);

        positionDisplay = selectedPosition == RobotConfig.StartPosition.CLOSE_SIDE
                ? ">>> CLOSE SIDE <<<" : "    CLOSE SIDE";
        telemetry.addData("          (D-Pad Right)", positionDisplay);

        telemetry.addData("", "");
        telemetry.addData("", "Press A to CONFIRM");
        telemetry.update();
    }

    private void displayConfirmation() {
        telemetry.clear();
        telemetry.addData("=== CONFIRM CONFIGURATION ===", "");
        telemetry.addData("", "");
        telemetry.addData("Alliance", selectedAlliance);
        telemetry.addData("Start Position", selectedPosition);
        telemetry.addData("", "");
        telemetry.addData("", "Press A to SAVE");
        telemetry.addData("", "Press B to GO BACK");
        telemetry.update();
    }

    private void updateButtonStates() {
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadRight = gamepad1.dpad_right;
        lastA = gamepad1.a;
        lastB = gamepad1.b;
    }
}