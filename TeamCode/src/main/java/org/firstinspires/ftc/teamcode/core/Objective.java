package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

public class Objective {
    private final DcMotor[] miMotors;
    private final Servo[] miServos;
    private final Task[] miTasks;
    private volatile boolean canRun = true;

    public Objective(DcMotor[] aiMotors, Servo[] aiServos, Task[] aiTasks) {
        miMotors = aiMotors;
        miServos = aiServos;
        miTasks = aiTasks;
    }

    /**
     * Method to execute the sequence of tasks this object stores
     * @author June
     * @param runtime input for handling time operations
     * @param abFloor boolean representing the touch sensor that checks if Arm_Extend has bottomed out
     */
    public void run(ElapsedTime runtime, boolean abFloor) {
        for (Task mtTask : miTasks) {
            if(!canRun) break;
            // Task
            // Telemetry
            System.out.println("Running task " + mtTask.name());

            // Set Value per motor
            for (int iMotor = 0; iMotor < miMotors.length; iMotor++) {
                if(!canRun) break;
                if (mtTask.motorPosition(iMotor) == -1) continue; // Skip if -1 (indicates a skip)
                DcMotor mdMotor = miMotors[iMotor];

                // If the current motor is "Arm_Extend", check to make sure the arm will not under-extend when this task is performed
                if (abFloor && Objects.equals(mdMotor.getDeviceName(), "Arm_Extend") && mtTask.motorPosition(iMotor) < 0)
                    continue;

                // Assign Power and Position
                double mdPower = mtTask.motorPower();
                mdMotor.setPower(mdPower);
                double mdPosition = mtTask.motorPosition(iMotor);
                mdMotor.setTargetPosition((int) mdPosition);

                // Make sure it is in RUN_TO_POSITION mode
                mdMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            for (int iServo = 0; iServo < miServos.length; iServo++) {
                if(!canRun) break;
                if (mtTask.servoPosition(iServo) == -1) continue; // Skip if -1 (indicates a skip)

                Servo miServo = miServos[iServo];

                // Apply Position
                miServo.setPosition(mtTask.servoPosition(iServo));
            }

            // Delay for set time after action
            while (isBusy(mtTask)) {
                if(!canRun) break;
                Thread.yield();
            }

            runtime.reset();

            double mdWaitTime = mtTask.waitFor() * mtTask.motorPower();
            while (runtime.seconds() < mdWaitTime) {
                if(!canRun) break;
                Thread.yield();
            }
        }
    }

    public void stop() {
        canRun = false;
        for (DcMotor motor : miMotors) {
            motor.setPower(0);
            motor.setTargetPosition(motor.getCurrentPosition());
        }
    }

    // Private Methods
    /**
     * Checks to see if each motor in the objective has reached its position or not
     * @author June
     * @param mtTask Task to check
     * @return boolean representing if the motor is busy
     */
    private boolean isBusy(Task mtTask) {
        for (int iMotor = 0; iMotor < miMotors.length; iMotor++) {
            DcMotor mdMotor = miMotors[iMotor];
            if(
                    mdMotor.isBusy() &&
                    mdMotor.getCurrentPosition() != mdMotor.getTargetPosition() &&
                    mdMotor.getCurrentPosition() != mtTask.motorPosition(iMotor)
            ) return true;

        }
        return false;
    }
}
