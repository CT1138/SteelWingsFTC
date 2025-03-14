package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Objects;

public class Task {
    private final double[] mdServoPositions;
    private final double[] mdMotorPositions;
    private final double[] mdMotorPowers;
    private final int miWaitFor;

    /**
     * Constructor for a single task.
     * @param adPositions 2D Array of servo and motor positions. First row is for servos, second row is for motors, third row is for motor POWERS
     * @param aiWaitFor Time in seconds to wait after proceeding to the next task
     */
    public Task(double[][] adPositions, int aiWaitFor) {
        mdServoPositions = adPositions[0];
        mdMotorPositions = adPositions[1];
        mdMotorPowers = adPositions[2];
        miWaitFor = aiWaitFor;
    }

    /**
     * Constructor for a single task.
     * @param adServoPositions Array of servo positions
     * @param aiMotorPositions Array of Motor Positions
     * @param adMotorPower Power to apply to motors
     * @param aiWaitFor
     */
    public Task(double[] adServoPositions, double[] aiMotorPositions, double adMotorPower, int aiWaitFor) {
        this.mdServoPositions = adServoPositions;
        this.mdMotorPositions = aiMotorPositions;
        this.mdMotorPowers = new double[] {adMotorPower};
        this.miWaitFor = aiWaitFor;
    }

    /**
     * Method to execute a singular task. Intended for use in LinearOpModes where it is less necessary to have the Objective class for handling sequential execution
     * @param aoMotors An array of motors your sequence of tasks will use. This can use as many or as few motors you want, depending on what you need this task to do.
     * @param aoServos An array of servos your sequence of tasks will use. This can use as many or as few motors you want, depending on what you need this task to do.
     * @param abFloor boolean representing the touch sensor that checks if Arm_Extend has bottomed out
     */
    public void run(DcMotor[] aoMotors, Servo[] aoServos, boolean abFloor) {
        // Set Value per motor
        for (int miMotorIndex = 0; miMotorIndex < aoMotors.length; miMotorIndex++) {
            if (this.motorPosition(miMotorIndex) == -1) continue; // Skip if -1 (indicates a skip)
            DcMotor mdMotor = aoMotors[miMotorIndex];

            // If the current motor is "Arm_Extend", check to make sure the arm will not under-extend when this task is performed
            if (abFloor && Objects.equals(mdMotor.getDeviceName(), "Arm_Extend") && this.motorPosition(miMotorIndex) < 0)
                continue;

            // Assign Power and Position
            double mdPower = this.motorPower(miMotorIndex);
            mdMotor.setPower(mdPower);
            double mdPosition = this.motorPosition(miMotorIndex);
            mdMotor.setTargetPosition((int) mdPosition);

            // Make sure it is in RUN_TO_POSITION mode
            mdMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        for (int miServoIndex = 0; miServoIndex < aoServos.length; miServoIndex++) {
            if (this.servoPosition(miServoIndex) == -1) continue; // Skip if -1 (indicates a skip)

            Servo miServo = aoServos[miServoIndex];

            // Apply Position
            miServo.setPosition(this.servoPosition(miServoIndex));
        }

        ElapsedTime moRuntime = new ElapsedTime();
        moRuntime.reset();

        double mdWaitTime = this.waitFor() * this.motorPower(0);
        while (moRuntime.seconds() < mdWaitTime) {
            Thread.yield();
        }
    }

    // Get name

    // Get Servo Positions
    public final double[] servoPositions() {
        return mdServoPositions;
    }
    public final double servoPosition(int aiIndex) {
        return mdServoPositions[aiIndex];
    }

    // Get Motor Positions
    public final double[] motorPositions() {
        return mdMotorPositions;
    }
    public final double motorPosition(int aiIndex) {
        return mdMotorPositions[aiIndex];
    }

    // Get Motor Power
    public final double[] motorPowers() {
        return mdMotorPowers;
    }
    public final double motorPower(int aiIndex) {
        return mdMotorPowers[aiIndex];
    }

    // Get Delay Time
    public final int waitFor() {
        return miWaitFor;
    }

    // Get the number of positions
    public final int length() {
        return mdMotorPositions.length + mdServoPositions.length;
    }

}
