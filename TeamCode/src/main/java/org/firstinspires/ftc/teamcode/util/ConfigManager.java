package org.firstinspires.ftc.teamcode.util;

import java.io.FileInputStream;
import java.io.IOException;
import java.util.Properties;

public class ConfigManager {

    private final Properties properties;

    // Constructor to load properties file
    public ConfigManager(String filePath) throws IOException {
        properties = new Properties();
        try (FileInputStream fis = new FileInputStream(filePath)) {
            properties.load(fis);
        } catch (IOException e) {
            System.err.println("Error loading config file: " + e.getMessage());
        }
    }

    // Get a String value
    public String getString(String key) {
        return properties.getProperty(key);
    }

    // Get a Double value
    public double getDouble(String key) {
        String value = properties.getProperty(key);
        return value != null ? Double.parseDouble(value) : 0.0;
    }

    // Get an Integer value
    public int getInt(String key) {
        String value = properties.getProperty(key);
        return value != null ? Integer.parseInt(value) : 0;
    }

    // Get a Float value
    public float getFloat(String key) {
        String value = properties.getProperty(key);
        return value != null ? Float.parseFloat(value) : 0.0f;
    }

    // Main method for testing
    public static void main(String[] args) {
        if (args.length == 0) {
            System.err.println("Usage: java ConfigManager <path_to_config.properties>");
            return;
        }

        try {
            // Create ConfigManager instance
            ConfigManager configManager = new ConfigManager(args[0]);

            // Example usage
            System.out.println("teamName: " + configManager.getString("teamName"));
            System.out.println("robotVersion: " + configManager.getDouble("robotVersion"));
            System.out.println("maxSpeed: " + configManager.getInt("maxSpeed"));
        } catch (IOException e) {
            System.err.println("Error reading properties file: " + e.getMessage());
        } catch (NumberFormatException e) {
            System.err.println("Error parsing numeric value: " + e.getMessage());
        }
    }
}
