package org.firstinspires.ftc.teamcode.base.config;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Static configuration manager that handles loading/saving robot config
 */
public class ConfigManager {
    private static final String CONFIG_FILE = "robot_config.json";
    private static RobotConfig currentConfig = null;
    private static final Gson gson = new GsonBuilder().setPrettyPrinting().create();

    /**
     * Load configuration from file
     * If file doesn't exist, creates default config
     */
    public static void load() {
        File file = getConfigFile();

        if (file.exists()) {
            try {
                String json = ReadWriteFile.readFile(file);
                currentConfig = gson.fromJson(json, RobotConfig.class);
            } catch (Exception e) {
                // If loading fails, create default
                currentConfig = new RobotConfig();
                save();
            }
        } else {
            // File doesn't exist, create default
            currentConfig = new RobotConfig();
            save();
        }
    }

    /**
     * Save current configuration to file
     */
    public static void save() {
        if (currentConfig == null) {
            currentConfig = new RobotConfig();
        }

        try {
            String json = gson.toJson(currentConfig);
            File file = getConfigFile();
            ReadWriteFile.writeFile(file, json);
        } catch (Exception e) {
            // Handle save error silently or log
        }
    }

    /**
     * Get the current configuration
     * Automatically loads if not already loaded
     */
    public static RobotConfig getConfig() {
        if (currentConfig == null) {
            load();
        }
        return currentConfig;
    }

    /**
     * Set and save new configuration
     */
    public static void setConfig(RobotConfig config) {
        currentConfig = config;
        save();
    }

    /**
     * Update alliance and save
     */
    public static void setAlliance(RobotConfig.Alliance alliance) {
        getConfig().setAlliance(alliance);
        save();
    }

    /**
     * Update start position and save
     */
    public static void setStartPosition(RobotConfig.StartPosition position) {
        getConfig().setStartPosition(position);
        save();
    }

    /**
     * Get the config file from the robot controller
     */
    private static File getConfigFile() {
        return new File(AppUtil.getInstance().getSettingsFile(CONFIG_FILE).getAbsolutePath());
    }

    /**
     * Reset to default configuration
     */
    public static void reset() {
        currentConfig = new RobotConfig();
        save();
    }
}