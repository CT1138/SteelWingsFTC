package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

public class Collection {
    // List to hold the tasks
    private ArrayList<Task> tasks;

    // Constructor to initialize the collection
    public Collection() {
        tasks = new ArrayList<>();
    }

    /**
     * Adds a new Task to the collection.
     * @param adPositions 2D array of servo, motor positions, and motor power(s)
     * @param aiWaitFor Time in seconds to wait after proceeding to the next task
     * @return The index of the newly added task
     */
    public int add(double[][] adPositions, int aiWaitFor) {
        // Create a new Task and add it to the list
        Task newTask = new Task(adPositions, aiWaitFor);
        tasks.add(newTask);

        // Return the index of the newly added task
        return tasks.size() - 1;
    }

    /**
     * Removes a Task from the collection based on its index.
     * @param index The index of the task to remove
     * @return True if the task was removed successfully, false otherwise
     */
    public boolean remove(int index) {
        if (index >= 0 && index < tasks.size()) {
            tasks.remove(index);
            return true;  // Task was removed successfully
        } else {
            return false;  // Index is out of bounds
        }
    }

    /**
     * Retrieve the Task at a specific index.
     * @param index The index of the task
     * @return The Task at the specified index
     */
    public Task getTask(int index) {
        return tasks.get(index);
    }

    /**
     * Returns an array of all the tasks in the collection.
     * @return An array of Task objects
     */
    public Task[] getTasks() {
        return tasks.toArray(new Task[0]);
    }

    /**
     * Returns the number of tasks in the collection.
     * @param aiMotors An array of motors your sequence of tasks will use. This can use as many or as few motors you want, depending on what you need this objective to do.
     * @param aiServos An array of servos your sequence of tasks will use. This can use as many or as few motors you want, depending on what you need this objective to do.
     * @return The number of tasks in the collection
     */
    public Objective buildObjective(DcMotor[] aiMotors, Servo[] aiServos) {
        return new Objective(aiMotors, aiServos, tasks.toArray(new Task[0]));
    }
}
