package org.firstinspires.ftc.teamcode.util;

import com.google.gson.Gson;
import com.google.gson.JsonSyntaxException;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

public class JSONreader {
    private final String filePath;
    private final Gson gson;

    public JSONreader(String filePath) {
        this.filePath = filePath;
        this.gson = new Gson();
    }

    public <T> T readData(Class<T> clazz) {
        try (FileReader reader = new FileReader(filePath)) {
            return gson.fromJson(reader, clazz);
        } catch (FileNotFoundException e) {
            System.err.println("File not found: " + e.getMessage());
        } catch (JsonSyntaxException e) {
            System.err.println("JSON syntax error: " + e.getMessage());
        } catch (IOException e) {
            System.err.println("IO error: " + e.getMessage());
        }
        return null;
    }
}
