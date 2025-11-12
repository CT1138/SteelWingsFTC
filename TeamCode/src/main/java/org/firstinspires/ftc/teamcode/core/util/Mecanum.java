package org.firstinspires.ftc.teamcode.core.util;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * The {@code Mecanum} class will automatically calculate the power values for each motor assuming the robot drivetrain is using mecanum wheels with 4WD
 */
public class Mecanum {
    private double mdDefaultSpeed;

    public Mecanum() {
        mdDefaultSpeed = 1.0;
    }
    public Mecanum(double adDefaultSpeed) {
        mdDefaultSpeed = adDefaultSpeed;
    }


    // inspired by this: https://github.com/brandon-gong/ftc-mecanum
    /**
     * Calculates the power values for each motor assuming the robot drivetrain is using mecanum wheels with 4WD
     * @param adDrive The forward/backward movement input
     * @param adStrafe The left/right movement input
     * @param adTwist The rotation movement input
     * @return Returns a double array for each motor in the order: Front Left, Front Right, Rear Left, Rear Right
     */

    public double[] Calculate(double adDrive, double adStrafe, double adTwist, boolean abBrake) {
        double mbFrontLeft = 0.0;
        double mbFrontRight = 0.0;
        double mbRearLeft = 0.0;
        double mbRearRight = 0.0;

        // Calculate Powers
        mbFrontLeft = adDrive + adStrafe + adTwist * (abBrake ? 1 : mdDefaultSpeed);
        mbFrontRight = adDrive - adStrafe - adTwist * (abBrake ? 1 : mdDefaultSpeed);
        mbRearLeft = adDrive - adStrafe + adTwist * (abBrake ? 1 : mdDefaultSpeed);
        mbRearRight = adDrive + adStrafe - adTwist * (abBrake ? 1 : mdDefaultSpeed);

        // Return Powers
        return new double[]{mbFrontLeft, mbFrontRight, mbRearLeft, mbRearRight};
    }
    public double[] Calculate(double adDrive, double adStrafe, double adTwist) {
        double mbFrontLeft = 0.0;
        double mbFrontRight = 0.0;
        double mbRearLeft = 0.0;
        double mbRearRight = 0.0;


        // Calculate Powers
        mbFrontLeft = adDrive + adStrafe + adTwist;
        mbFrontRight = adDrive - adStrafe - adTwist;
        mbRearLeft = adDrive - adStrafe + adTwist;
        mbRearRight = adDrive + adStrafe - adTwist;

        // Return Powers
        return new double[]{mbFrontLeft, mbFrontRight, mbRearLeft, mbRearRight};
    }

    /**
     * Check if any of the motors are busy
     * @param aoMotors Array of motors to check
     * @return Returns true if they are busy, false if they are not
     */
    public boolean areMotorsBusy(DcMotor[] aoMotors) {
        for (DcMotor moMotor : aoMotors) {
            if (!moMotor.isBusy()) {
                return true;
            }
        }
        return false;
    }
}
