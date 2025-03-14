package org.firstinspires.ftc.teamcode.core.util;

import org.firstinspires.ftc.teamcode.core.Task;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class CSVManager {

    // Method to read CSV and convert to Task objects
    public Task[] toTasks(String filePath, boolean hasHeader) {
        List<Task> taskList = new ArrayList<>();
        BufferedReader br = null;
        String line = "";

        try {
            br = new BufferedReader(new FileReader(filePath));

            while ((line = br.readLine()) != null) {
                // Skip header row if present
                if (hasHeader) {
                    hasHeader = false;
                    continue;
                }

                // Split by commas (assuming CSV format), but parse sub-arrays separately
                String[] row = line.split(",");

                // Parse the values from the row (handling space-separated arrays)
                double[] adServoPositions = parseDoubleArray(row[0].trim()); // First column
                double[] aiMotorPositions = parseDoubleArray(row[1].trim()); // Second column
                double adMotorPower = Double.parseDouble(row[2].trim()); // Third column
                int aiWaitFor = Integer.parseInt(row[3].trim()); // Fourth column

                // Create and add a new Task object
                taskList.add(new Task(adServoPositions, aiMotorPositions, adMotorPower, aiWaitFor));
            }
        } catch (IOException e) {
            e.printStackTrace();
        } finally {
            try {
                if (br != null) {
                    br.close();
                }
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        return taskList.toArray(new Task[0]);
    }

    // Helper method to parse the CSV string to a double array (e.g. "1.2, 3.4, 5.6")
    private double[] parseDoubleArray(String str) {
        String[] tokens = str.split(" "); // Assuming spaces separate numbers
        double[] result = new double[tokens.length];
        for (int i = 0; i < tokens.length; i++) {
            result[i] = Double.parseDouble(tokens[i]);
        }
        return result;
    }

}