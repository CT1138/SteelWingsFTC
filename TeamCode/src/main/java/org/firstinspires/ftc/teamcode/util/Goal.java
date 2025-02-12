package org.firstinspires.ftc.teamcode.util;

import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Objects;

public class Goal {
    private String mfNname;
    private Servo[] msServos;
    private DcMotor[] mmMotors;
    private int[] miPositions;

    private double[] mdPowers;

    public Goal(String asName, DcMotor[] amMotors, Servo[] asServos, int[] aiPositions, double[] aiPowers) {
        mfNname = asName;
        mmMotors = amMotors;
        miPositions = aiPositions;
        msServos = asServos;
        mdPowers = aiPowers;
    }

    public void RunToGoal(boolean mbFloor, double mdPowerModifier) {
        if(mmMotors == null || msServos == null || miPositions ==  null || mdPowers == null) return;

        for (int i = 0; i < mmMotors.length; i++) {
            double mdFinalPower = mdPowers[i] * mdPowerModifier;
            mmMotors[i].setPower(mdFinalPower);
            mmMotors[i].setTargetPosition(miPositions[i]);

            // failsafe to ensure the arm does not underextend
            if(Objects.equals(mmMotors[i].getDeviceName(), "Arm_Extend") && mbFloor) mmMotors[i].setPower(0);

            mmMotors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        for (int i = 0; i < msServos.length; i++) {
            msServos[i].setPosition(miPositions[mmMotors.length + i]);
        }
    }

    public boolean isBusy() {
        if(mmMotors == null || miPositions == null) return false;

        if (mmMotors.length + msServos.length != miPositions.length) {
            throw new IllegalStateException("Mismatch between motor and servo arrays and position array");
        }
        for (int i = 0; i < mmMotors.length; i++) {
            if (mmMotors[i].getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                if (mmMotors[i].isBusy()) {
                    return true;
                }
            } else {
                if (mmMotors[i].getCurrentPosition() != miPositions[i]) {
                    return true;
                }
            }
        }
        return false;
    }

    @NonNull
    @Override
    public String toString() {
        return mfNname;
    }
}
