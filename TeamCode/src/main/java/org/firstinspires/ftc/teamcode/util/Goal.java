package org.firstinspires.ftc.teamcode.util;

import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Goal {
    private Servo[] servos;
    private DcMotor[] motors;
    private int[] positions;

    public Goal(DcMotor[] m_motors, Servo[] s_servo, int[] iPositions) {
        this.motors = m_motors;
        this.positions = iPositions;
        this.servos = s_servo;
    }

    public void RunToGoal(double power, double waitAfter) {
        for (int i = 0; i < motors.length; i++) {
            motors[i].setPower(power);
            motors[i].setTargetPosition(positions[i]);
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        for (int i = 0; i < servos.length; i++) {
            servos[i].setPosition(positions[motors.length + i]);
        }
    }

    public boolean isBusy() {
        if (motors.length != positions.length) {
            throw new IllegalStateException("Motor and target position arrays must be the same size");
        }
        for (int i = 0; i < motors.length; i++) {
            if (motors[i].getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                if (motors[i].isBusy()) {
                    return true;
                }
            } else {
                if (motors[i].getCurrentPosition() != positions[i]) {
                    return true;
                }
            }
        }
        return false;
    }
}
