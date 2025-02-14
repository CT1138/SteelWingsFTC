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

    public Objective(DcMotor[] aiMotors, Servo[] aiServos, Task[] aiTasks) {
        miMotors = aiMotors;
        miServos = aiServos;
        miTasks = aiTasks;
    }

    public void run(ElapsedTime runtime, Telemetry telemetry, boolean abFloor) {
        runtime.reset();

        for (Task mtTask : miTasks) {
            // Task
            // Telemetry
            telemetry.addLine("Running task " + mtTask.name());
            System.out.println("Running task " + mtTask.name());

            // Set Value per motor
            for (int iMotor = 0; iMotor < miMotors.length; iMotor++) {
                if (mtTask.motorPosition(iMotor) == -1) continue; // Skip if -1 (indicates a skip)
                DcMotor mdMotor = miMotors[iMotor];

                // If the current motor is "Arm_Extend", check to make sure the arm will not under-extend when this task is performed
                if (abFloor && Objects.equals(mdMotor.getDeviceName(), "Arm_Extend") && mtTask.motorPosition(iMotor) < 0)
                    continue;

                // Assign Power and Position
                double mdPower = mtTask.motorPower(iMotor);
                mdMotor.setPower(mdPower);
                double mdPosition = mtTask.motorPosition(iMotor);
                mdMotor.setTargetPosition((int) mdPosition);

                // Make sure it is in RUN_TO_POSITION mode
                mdMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            for (int iServo = 0; iServo < miServos.length; iServo++) {
                if (mtTask.servoPosition(iServo) == -1) continue; // Skip if -1 (indicates a skip)

                Servo miServo = miServos[iServo];

                // Apply Position
                miServo.setPosition(mtTask.servoPosition(iServo));
            }

            // Delay for set time after action
            while (runtime.seconds() < mtTask.waitFor()) {
                telemetry.addLine("Running goal.... delay: " + (mtTask.waitFor() - runtime.seconds()) );
            }
        }
    }
}
