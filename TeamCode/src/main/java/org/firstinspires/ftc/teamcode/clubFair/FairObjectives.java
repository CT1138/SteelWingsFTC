package org.firstinspires.ftc.teamcode.clubFair;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.core.Task;
import org.firstinspires.ftc.teamcode.core.Objective;

public class FairObjectives {
    private final DcMotor[] moMotors;
    private final Servo[] moServos;

    // Positions for Reference
    private final int miElbowUp = 860;
    private final int miElbowForward = 450;
    private final int miElbowAngled = 760;
    private final int miWristLeftMax = -140;
    private final int miWristRightMax = 140;
    private final int miExtendMax = 4100;

    public FairObjectives(DcMotor[] aoMotors, Servo[] aoServos) {
        moMotors = aoMotors;
        moServos = aoServos;
    }

    // ========================
    // BEGIN OBJECTIVES & TASKS
    // ========================


    // Say Hi!
    public final Objective foWave(double adArmPower) {
        // Build Tasks Array
        Task[] moTasks = {
         // new Task(name,                  servoPositions,     motorPositions,            motorPowers,     waitFor)
            new Task("Raise Arm",   new double[]{1.0},  new int[]{100, -1, -1},    adArmPower, 4),

            new Task("Wave Up",     new double[]{0},    new int[]{-1, 430, 100},     adArmPower * 0.5, 0),
            new Task("Wave down",     new double[]{1.0},  new int[]{-1, 560, -100},    adArmPower * 0.5, 0),
            new Task("Wave Up",     new double[]{0},    new int[]{-1, 430, 100},     adArmPower * 0.5, 0),
            new Task("Wave down",     new double[]{1.0},  new int[]{-1, 560, -100},    adArmPower * 0.5, 0),
            new Task("Wave Up",     new double[]{0},    new int[]{-1, 430, 100},     adArmPower * 0.5, 0),
            new Task("Wave down",     new double[]{1.0},  new int[]{-1, 560, -100},    adArmPower * 0.5, 0),

            new Task("Hold Arm",    new double[]{1.0},  new int[]{-1, -1, -1},       adArmPower, 2),

            new Task("Zero Arm",    new double[]{1.0},  new int[]{0, 0, 0},          adArmPower, 0)
        };

        // Return the array of tasks as an objective object
        return new Objective(moMotors, moServos, moTasks);
    }

    // Grab Sample From Floor
    public final Objective foGrabFromFloor(double adArmPower) {

        Task[] moTasks = {
            new Task("Get into Position", new double[]{0.0}, new int[]{0, 0, 0}, adArmPower, 0),
            new Task("Grab", new double[]{1.0}, new int[]{-1, -1, -1}, adArmPower, 1)
        };

        return new Objective(moMotors, moServos, moTasks);
    }

    // Grab Sample From Lower Rung
    public final Objective foGrabFromLowerRung(double adArmPower) {
        Task[] moTasks = {
            new Task("Get into Position", new double[]{0.0}, new int[]{0, miElbowForward, 0}, adArmPower, 0),
            new Task("Grab", new double[]{1.0}, new int[]{-1, -1, -1}, adArmPower, 1),
            new Task("Raise While Holding it", new double[]{1.0}, new int[]{200, -1, -1}, adArmPower, 0),
            new Task("Return to Zero", new double[]{1.0}, new int[]{0, 0, 0}, adArmPower, 0)
        };

        return new Objective(moMotors, moServos, moTasks);
    }

    // Return to Zero Positions
    public final Objective foZero(double aoArmPower) {
        // Build Tasks Array
        Task[] moTasks = {
         // new Task(name,                  servoPositions,      motorPositions,        motorPower       waitFor)
            new Task("Zero Arm",   new double[]{-1.0},  new int[]{0, 0, 0},    aoArmPower,   0),
        };
        return new Objective(moMotors, moServos, moTasks);
    }

    // ======================
    // END OBJECTIVES & TASKS
    // ======================
}
