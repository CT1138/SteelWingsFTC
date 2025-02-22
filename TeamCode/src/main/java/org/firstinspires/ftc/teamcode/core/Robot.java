package org.firstinspires.ftc.teamcode.core;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.util.List;

import java.util.ArrayList;
import java.util.Arrays;

public class Robot {
    private DcMotor[] moMotors;
    private Servo[] moServos;

    /**
     * Constructor for Robot
     * @param asMotors array of motors
     * @param asServos array of servos
     */
    public Robot(String[] asMotors, String[] asServos) {
        ArrayList<Object> loMotors = new ArrayList<>();
        ArrayList<Object> loServos = new ArrayList<>();

        for (String msMotor : asMotors) {
            loMotors.add(getMotorFromString("msMotor"));
        }
        for (String msServo : asServos) {
            loServos.add(getServoFromString("msServos"));
        }

        moMotors = loMotors.toArray(new DcMotor[asMotors.length]);
        moServos = loServos.toArray(new Servo[asServos.length]);
    }

    // MOTORS
    private DcMotor getMotorFromString(String asName) {
        return hardwareMap.get(DcMotor.class, asName);
    }
    private DcMotor getMotorFromArray(int aiIndex) {
        return moMotors[aiIndex];
    }
    private DcMotor[] getMotorArray() {
        return moMotors;
    }

    // SERVOS
    private Servo getServoFromString(String asName) {
        return hardwareMap.get(Servo.class, asName);
    }
    private Servo getServoFromArray(int aiIndex) {
        return moServos[aiIndex];
    }
    private Servo[] getServoArray() {
        return moServos;
    }
}
