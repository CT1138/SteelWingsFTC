package org.firstinspires.ftc.teamcode.core.util;

public class PIDcontroller {
    public double kP, kI, kD;
    private double integral, lastError;

    public PIDcontroller(double p, double i, double d) {
        // --- PID Controller Gain Values ---
        // kP (Proportional Gain):
        //     Controls how strongly the system reacts to the *current* error.
        //     Higher kP → faster correction, but too high can cause oscillation (robot overshoots or wiggles).
        kP = p;

        // kI (Integral Gain):
        //     Controls how strongly the system reacts to *accumulated past* error over time.
        //     Useful for eliminating small constant drift (like uneven wheel friction).
        //     Often left small or zero in FTC, since it can make motion unstable if too high.
        kI = i;

        // kD (Derivative Gain):
        //     Reacts to the *rate of change* of the error (how quickly error is changing).
        //     Acts like damping — it smooths out the response and prevents overshoot.
        //     Too high can make movement sluggish or jittery.
        kD = d;
    }

    public void adjust(double aP, double aI, double aD) {
        kP = aP;
        kI = aI;
        kD = aD;
    }

    public double update(double error, double dt) {
        integral += error * dt;
        double derivative = (error - lastError) / dt;
        double output = kP * error + kI * integral + kD * derivative;
        lastError = error;
        return output;
    }

    public void reset() {
        integral = 0;
        lastError = 0;
    }
}
