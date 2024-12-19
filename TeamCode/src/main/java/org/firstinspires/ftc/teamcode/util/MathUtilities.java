package org.firstinspires.ftc.teamcode.util;

public class MathUtilities {
    public int percentToPosition(int MIN, int MAX, double percent) {
        double range = MAX - MIN;
        return (int) Math.round(MIN + (range * percent));
    }
}
