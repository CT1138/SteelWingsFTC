package org.firstinspires.ftc.teamcode.clubFair;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.core.Task;
import org.firstinspires.ftc.teamcode.core.Objective;

public class fairObjectives {
    private final DcMotor[] miMotors;
    private final Servo[] miServos;
    public fairObjectives(DcMotor[] aiMotors, Servo[] aiServos, Task[] miTasks) {
        miMotors = aiMotors;
        miServos = aiServos;
    }

    // BEGIN OBJECTIVES & TASKS
    public final Objective foWave() {

        return null;
    }
}
