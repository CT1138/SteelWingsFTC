package org.firstinspires.ftc.teamcode.core;

public class Task {
    private final String mfNname;
    private final double[] mdServoPositions;
    private final int[] mdMotorPositions;
    private final double[] mdMotorPowers;
    private final int miWaitFor;

    public Task(String asName, double[] adServoPositions, int[] aiMotorPositions, double[] adMotorPowers, int aiWaitFor) {
        mfNname = asName;
        mdServoPositions = adServoPositions;
        mdMotorPositions = aiMotorPositions;
        mdMotorPowers = adMotorPowers;
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

    // Get Motor Powers
    public final double[] motorPowers() {
        return mdMotorPowers;
    }
    public final double motorPower(int miIndex) {
        return mdMotorPowers[miIndex];
    }

    // Get Delay Time
    public final int waitFor() {
        return miWaitFor;
    }
}
