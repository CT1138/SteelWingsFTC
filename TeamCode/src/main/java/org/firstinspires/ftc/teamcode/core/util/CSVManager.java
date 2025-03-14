package org.firstinspires.ftc.teamcode.core.util;

import org.firstinspires.ftc.teamcode.core.Task;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class CSVManager {

    // Method to read CSV and convert to Task objects
    public Task[] toTasks(String filePath) {
        List<Task> taskList = new ArrayList<>(); // To store Task objects
        BufferedReader br = null;
        String line = "";
        String csvSplitBy = ","; // Define the delimiter

        try {
            br = new BufferedReader(new FileReader(filePath));
            while ((line = br.readLine()) != null) {
                // Split the row into individual values
                String[] row = line.split(csvSplitBy);

                // Parse the values from the row
                double[] adServoPositions = parseDoubleArray(row[0]);
                double[] aiMotorPositions = parseDoubleArray(row[1]);
                double adMotorPower = Double.parseDouble(row[2]);
                int aiWaitFor = Integer.parseInt(row[3]);

                // Create a new Task object and add it to the list
                taskList.add(new Task(adServoPositions, aiMotorPositions, adMotorPower, aiWaitFor));
            }
        } catch (IOException e) {
            e.printStackTrace();
        } finally {
            try {
                if (br != null) {
                    br.close(); // Close the BufferedReader after use
                }
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        // Convert the List to an array and return it
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