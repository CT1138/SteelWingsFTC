package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Task {
    private final String mfNname;
    private final double[] mdServoPositions;
    private final int[] mdMotorPositions;
    private final double mdMotorPower;
    private final int miWaitFor;

    public Task(String asName, double[] adServoPositions, int[] aiMotorPositions, double adMotorPower, int aiWaitFor) {
        mfNname = asName;
        mdServoPositions = adServoPositions;
        mdMotorPositions = aiMotorPositions;
        mdMotorPower = adMotorPower;
        miWaitFor = aiWaitFor;
    }

    // Get name
    public String name() {
        return mfNname;
    }

    // Get Servo Positions
    public final double[] servoPositions() {
        return mdServoPositions;
    }
    public final double servoPosition(int miIndex) {
        return mdServoPositions[miIndex];
    }

    // Get Motor Positions
    public final int[] motorPositions() {
        return mdMotorPositions;
    }
    public final int motorPosition(int miIndex) {
        return mdMotorPositions[miIndex];
    }

    // Get Motor Power
    public final double motorPower() {
        return mdMotorPower;
    }

    // Get Delay Time
    public final int waitFor() {
        return miWaitFor;
    }
}
