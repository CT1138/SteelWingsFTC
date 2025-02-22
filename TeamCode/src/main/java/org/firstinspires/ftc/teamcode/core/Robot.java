package org.firstinspires.ftc.teamcode.core;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.Objects;

public class Robot {
    private final DcMotor[] moMotors;
    private final String[] msMotors;
    private final Servo[] moServos;
    private final String[] msServos;

    /**
     * Constructor for Robot
     * @param asMotors array of motors
     * @param asServos array of servos
     */
    public Robot(String[] asMotors, String[] asServos) {
        msMotors = asMotors;
        msServos = asServos;

        ArrayList<DcMotor> loMotors = new ArrayList<>();
        ArrayList<Servo> loServos = new ArrayList<>();

        for (String msMotor : asMotors) {
            loMotors.add(hardwareMotor(msMotor));
        }
        for (String msServo : asServos) {
            loServos.add(hardwareServo(msServo));
        }

        moMotors = loMotors.toArray(new DcMotor[asMotors.length]);
        moServos = loServos.toArray(new Servo[asServos.length]);
    }

    // MOTORS
    private DcMotor hardwareMotor(String asName) {
        return hardwareMap.get(DcMotor.class, asName);
    }
    public DcMotor motor(int aiIndex) {
        return moMotors[aiIndex];
    }
    public DcMotor motor(String asName) {
        int miIndex = -1;
        for (int i = 0; i < msMotors.length; i++) {
            if(!Objects.equals(msMotors[i], asName)) continue;
            miIndex = i;
            break;
        }
        if(miIndex == -1) return null;
        return moMotors[miIndex];
    }
    public DcMotor[] getMotorArray() {
        return moMotors;
    }

    // SERVOS
    private Servo hardwareServo(String asName) {
        return hardwareMap.get(Servo.class, asName);
    }
    public Servo servo(int aiIndex) {
        return moServos[aiIndex];
    }
    public Servo servo(String asName) {
        int miIndex = -1;
        for (int i = 0; i < msServos.length; i++) {
            if(!Objects.equals(msServos[i], asName)) continue;
            miIndex = i;
            break;
        }
        if(miIndex == -1) return null;
        return moServos[miIndex];
    }
    public Servo[] getServoArray() {
        return moServos;
    }
}
