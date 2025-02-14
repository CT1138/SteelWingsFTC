package org.firstinspires.ftc.teamcode.clubFair;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.core.Task;
import org.firstinspires.ftc.teamcode.core.Objective;

public class FairObjectives {
    private final DcMotor[] miMotors;
    private final Servo[] miServos;

    // Positions for Reference
    private final int miElbowUp = 860;
    private final int miElbowForward = 450;
    private final int miElbowAngled = 760;
    private final int miWristLeftMax = -140;
    private final int miWristRightMax = 140;
    private final int miExtendMax = 4100;

    public FairObjectives(DcMotor[] aiMotors, Servo[] aiServos) {
        miMotors = aiMotors;
        miServos = aiServos;
    }

    // ========================
    // BEGIN OBJECTIVES & TASKS
    // ========================


    // Say Hi!
    public final Objective foWave(double mdArmPower) {
        // Build Tasks Array
        Task[] miTasks = {
         // new Task(name,                  servoPositions,     motorPositions,            motorPowers,     waitFor)
            new Task("Raise Arm",   new double[]{1.0},  new int[]{200, -1, -1},    mdArmPower, 4),

            new Task("Wave Up",     new double[]{0},    new int[]{-1, 430, 100},     mdArmPower * 0.5, 0),
            new Task("Wave down",     new double[]{1.0},  new int[]{-1, 560, -100},    mdArmPower * 0.5, 0),
            new Task("Wave Up",     new double[]{0},    new int[]{-1, 430, 100},     mdArmPower * 0.5, 0),
            new Task("Wave down",     new double[]{1.0},  new int[]{-1, 560, -100},    mdArmPower * 0.5, 0),
            new Task("Wave Up",     new double[]{0},    new int[]{-1, 430, 100},     mdArmPower * 0.5, 0),
            new Task("Wave down",     new double[]{1.0},  new int[]{-1, 560, -100},    mdArmPower * 0.5, 0),

            new Task("Hold Arm",    new double[]{1.0},  new int[]{-1, -1, -1},       mdArmPower, 2),

            new Task("Zero Arm",    new double[]{1.0},  new int[]{0, 0, 0},          mdArmPower, 0)
        };

        // Return the array of tasks as an objective object
        return new Objective(miMotors, miServos, miTasks);
    }

    // Grab Sample From Floor
    public final Objective foGrabFromFloor(double mdArmPower) {

        Task[] miTasks = {
            new Task("Get into Position", new double[]{0.0}, new int[]{0, 0, 0}, mdArmPower, 0),
            new Task("Grab", new double[]{1.0}, new int[]{-1, -1, -1}, mdArmPower, 1)
        };

        return new Objective(miMotors, miServos, miTasks);
    }

    // Grab Sample From Lower Rung
    public final Objective foGrabFromLowerRung(double mdArmPower) {
        Task[] miTasks = {
            new Task("Get into Position", new double[]{0.0}, new int[]{0, miElbowForward, 0}, mdArmPower, 0),
            new Task("Grab", new double[]{1.0}, new int[]{-1, -1, -1}, mdArmPower, 1),
            new Task("Raise While Holding it", new double[]{1.0}, new int[]{200, -1, -1}, mdArmPower, 0),
            new Task("Return to Zero", new double[]{1.0}, new int[]{0, 0, 0}, mdArmPower, 0)
        };

        return new Objective(miMotors, miServos, miTasks);
    }

    // Return to Zero Positions
    public final Objective foZero(double mdArmPower) {
        // Build Tasks Array
        Task[] miTasks = {
         // new Task(name,                  servoPositions,      motorPositions,        motorPower       waitFor)
            new Task("Raise Arm",   new double[]{-1.0},  new int[]{0, 0, 0},    mdArmPower,   0),
        };
        return new Objective(miMotors, miServos, miTasks);
    }

    // ======================
    // END OBJECTIVES & TASKS
    // ======================
}
