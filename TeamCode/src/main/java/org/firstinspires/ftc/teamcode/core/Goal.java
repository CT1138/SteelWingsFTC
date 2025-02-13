package org.firstinspires.ftc.teamcode.core;

import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Objects;

public class Goal {
    private final String mfNname;
    private final Servo[] msServos;
    private final DcMotor[] mmMotors;
    private final int[] miPositions;

    private final double[] mdPowers;

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

            // Do not change anything if the power or position is -1 (indicates a skip)
            if (mdPowers[i] == -1 || miPositions[i] == -1) continue;

            // Set the power and positions if all else is clear
            double mdFinalPower = mdPowers[i] * mdPowerModifier;
            if(!Objects.equals(mmMotors[i].getDeviceName(), "Arm_Extend") && !mbFloor) mmMotors[i].setPower(mdFinalPower);
            mmMotors[i].setTargetPosition(miPositions[i]);
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
