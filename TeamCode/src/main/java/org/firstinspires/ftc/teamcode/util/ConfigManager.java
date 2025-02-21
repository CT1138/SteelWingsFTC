package org.firstinspires.ftc.teamcode.util;

import java.io.FileInputStream;
import java.io.IOException;
import java.util.Properties;

public class ConfigManager {

    private final Properties moProperties;

    // Constructor to load properties file
    public ConfigManager(String asFilePath) throws IOException {
        moProperties = new Properties();
        try (FileInputStream moFIS = new FileInputStream(asFilePath)) {
            moProperties.load(moFIS);
        } catch (IOException e) {
            System.err.println("Error loading config file: " + e.getMessage());
        }
    }

    // Get a String value
    public String getString(String asKey) {
        return moProperties.getProperty(asKey);
    }

    // Get a Double value
    public double getDouble(String asKey) {
        String value = moProperties.getProperty(asKey);
        return value != null ? Double.parseDouble(value) : 0.0;
    }

    // Get an Integer value
    public int getInt(String asKey) {
        String value = moProperties.getProperty(asKey);
        return value != null ? Integer.parseInt(value) : 0;
    }

    // Get a Float value
    public float getFloat(String asKey) {
        String value = moProperties.getProperty(asKey);
        return value != null ? Float.parseFloat(value) : 0.0f;
    }
}
