package org.firstinspires.ftc.teamcode.util;

public class Mecanum {
    public double[] calculate(double drive, double strafe, double twist, boolean uncap) {
        double frontleft = 0.0;
        double frontright = 0.0;
        double rearleft = 0.0;
        double rearright = 0.0;

        if (uncap) {
            double modifier = 1;
            drive = drive * modifier;
            strafe = strafe * modifier;
            twist = twist * modifier;
        }

        // Calculate Powers
        frontleft = drive + strafe + twist;
        frontright = drive - strafe - twist;
        rearleft = drive - strafe + twist;
        rearright = drive + strafe + twist;

        // Return Powers
        return new double[]{frontleft, frontright, rearleft, rearright};
    }
}
