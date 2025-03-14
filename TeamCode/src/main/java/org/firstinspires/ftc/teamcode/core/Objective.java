package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Objects;

public class Objective {
    private final DcMotor[] moMotors;
    private final Servo[] moServos;
    private final Task[] moTasks;
    private volatile boolean mbCanRun = true;

    /**
     * Constructor for Objective
     * @param aoMotors An array of motors your sequence of tasks will use. This can use as many or as few motors you want, depending on what you need this objective to do.
     * @param aoServos An array of servos your sequence of tasks will use. This can use as many or as few motors you want, depending on what you need this objective to do.
     * @param aoTasks An array of tasks your sequence of tasks will use. Each task must have the same amount of motors and servos as the rest, an error will be thrown if there is a mismatch.
     */
    public Objective(DcMotor[] aoMotors, Servo[] aoServos, Task[] aoTasks) {
        moMotors = aoMotors;
        moServos = aoServos;
        moTasks = aoTasks;

        if(!sequenceIsValid()) throw new IllegalArgumentException("Sequence is not valid");
    }

    /**
     * Method to execute the sequence of tasks this object stores
     * @author June
     * @param aoRuntime input for handling time operations
     * @param abFloor boolean representing the touch sensor that checks if Arm_Extend has bottomed out
     */
    public void run(ElapsedTime aoRuntime, boolean abFloor) {
        for (Task moTask : moTasks) {
            if(!mbCanRun) break;
            // Task
            // Set Value per motor
            for (int miMotorIndex = 0; miMotorIndex < moMotors.length; miMotorIndex++) {
                if(!mbCanRun) break;
                if (moTask.motorPosition(miMotorIndex) == -1) continue; // Skip if -1 (indicates a skip)
                DcMotor mdMotor = moMotors[miMotorIndex];

                // If the current motor is "Arm_Extend", check to make sure the arm will not under-extend when this task is performed
                if (abFloor && Objects.equals(mdMotor.getDeviceName(), "Arm_Extend") && moTask.motorPosition(miMotorIndex) < 0)
                    continue;

                // Assign Power and Position
                double mdPower = moTask.motorPower(0);
                mdMotor.setPower(mdPower);
                double mdPosition = moTask.motorPosition(miMotorIndex);
                mdMotor.setTargetPosition((int) mdPosition);

                // Make sure it is in RUN_TO_POSITION mode
                mdMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            for (int miServoIndex = 0; miServoIndex < moServos.length; miServoIndex++) {
                if(!mbCanRun) break;
                if (moTask.servoPosition(miServoIndex) == -1) continue; // Skip if -1 (indicates a skip)

                Servo miServo = moServos[miServoIndex];

                // Apply Position
                miServo.setPosition(moTask.servoPosition(miServoIndex));
            }

            // Delay for set time after action
            while (isBusy(moTask)) {
                if(!mbCanRun) break;
                Thread.yield();
            }

            aoRuntime.reset();

            double mdWaitTime = moTask.waitFor() * moTask.motorPower(0);
            while (aoRuntime.seconds() < mdWaitTime) {
                if(!mbCanRun) break;
                Thread.yield();
            }
        }
    }

    /**
     * Method to check to see if each motor in the objective has reached its position or not
     * @return if any motors are busy or not at their target position, return false. Otherwise, return true.
     */
    public boolean canMoveOn() {
        for (DcMotor moMotor : moMotors) if (moMotor.isBusy() && moMotor.getCurrentPosition() != moMotor.getTargetPosition()) return false;
        return true;
    }

    /**
     * Method to stop the sequence while executing
     */
    public void stop() {
        mbCanRun = false;
        for (DcMotor motor : moMotors) {
            motor.setPower(0);
            motor.setTargetPosition(motor.getCurrentPosition());
        }
        for (Servo servo : moServos) {
            servo.setPosition(servo.getPosition());
        }
    }

    // Private Methods
    /**
     * Checks to see if each motor in the objective has reached its position or not
     * @author June
     * @param moTask Task to check
     * @return boolean representing if the motor is busy
     */
    private boolean isBusy(Task moTask) {
        for (int iMotor = 0; iMotor < moMotors.length; iMotor++) {
            DcMotor moMotor = moMotors[iMotor];
            if(
                    moMotor.isBusy() &&
                    moMotor.getCurrentPosition() != moMotor.getTargetPosition() &&
                    moMotor.getCurrentPosition() != moTask.motorPosition(iMotor)
            ) return true;

        }
        return false;
    }

    /**
     * Checks to see if the sequence of tasks is valid based off of the motors and servos in the objective
     * @return boolean representing if the sequence is valid
     */
    private boolean sequenceIsValid() {
        int miTaskLength = -1;
        boolean mbIsValid = true;

        for (Task moTask : moTasks) {
            // Check if the Tasks array has the same motors and servos as the Objective object
                if(moMotors.length + moServos.length != moTask.length()) return false;

            // Check to make sure each Task in the Sequence has an array of Positions proportional to the amount of motors and servos
                int miCurrentLength = moTask.motorPositions().length + moTask.servoPositions().length;

                // If miTaskLength is -1, assume that this is the first task we have checked
                if (miTaskLength == -1) {
                    // Store the length of the current task to miTaskLength for future comparison
                    miTaskLength = miCurrentLength;
                    continue;
                }


                // If there is a mismatch, report false
                if (miTaskLength != miCurrentLength) {
                    mbIsValid = false;
                    break;
                }
        }

        return mbIsValid;
    }
}
