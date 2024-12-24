package org.firstinspires.ftc.teamcode.util;

import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Goal {
    private Servo[] Servos;
    private DcMotor[] Motors;
    private int[] Positions;

    public Goal(DcMotor[] m_motors, Servo[] s_servo, int[] iPositions) {
        this.Motors = m_motors;
        this.Positions = iPositions;
        this.Servos = s_servo;
    }

    public void RunToGoal(double power, double waitAfter) {
        for (int i = 0; i < Motors.length; i++) {
            Motors[i].setPower(power);
            Motors[i].setTargetPosition(Positions[i]);
            Motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        for (int i = 0; i < Servos.length; i++) {
            Servos[i].setPosition(Positions[Motors.length + i]);
        }
    }
}
