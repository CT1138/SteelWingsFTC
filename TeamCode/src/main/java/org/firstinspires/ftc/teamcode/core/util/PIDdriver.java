package org.firstinspires.ftc.teamcode.core.util;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.core.util.Mecanum;

public class PIDdriver {
    private IMU imu;
    private Mecanum mecanum;
    public PIDcontroller pid;

    private double targetHeading = 0;
    private ElapsedTime pidTimer = new ElapsedTime();

    public PIDdriver(IMU imu, Mecanum mecanum, double kP, double kI, double kD) {
        this.imu = imu;
        this.mecanum = mecanum;
        this.pid = new PIDcontroller(kP, kI, kD);
    }

    public double[] Calculate(double adDrive, double adStrafe, double adTwist, boolean abUncap) {
        double currentHeading = getHeading();
        double mdTwist = adTwist;

        // Apply PID heading correction when driving mostly straight
        if (Math.abs(adDrive) > 0.05 && Math.abs(adTwist) < 0.05) {
            if (pidTimer.seconds() > 0.05) { // small delay to control PID loop frequency
                double error = normalizeAngle(targetHeading - currentHeading);
                double correction = pid.update(error, pidTimer.seconds());
                pidTimer.reset();
                mdTwist = correction;
            }
        } else {
            // When rotating or idle, update target heading and reset PID
            targetHeading = currentHeading;
            pid.reset();
            pidTimer.reset();
        }

        // Calculate wheel powers using Mecanum class
        return mecanum.Calculate(mdTwist, adStrafe, -adDrive, abUncap);
    }

    private double getHeading() {
        // Returns the yaw (rotation around vertical axis) in degrees
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    // Heading angle cap
    private double normalizeAngle(double angle) {
        return (angle + 540) % 360 - 180;
    }
}
