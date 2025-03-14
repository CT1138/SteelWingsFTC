package org.firstinspires.ftc.teamcode.core.util;

/**
 * This Class can store various math operations you may find yourself using often or in multiple classes.
 */
public class MathUtilities {
    /**
     * Converts a percentage to a position. If aiMinimum = 0 and aiMaximum = 100 and adPercent = 0.5, then this will return 50.
     * @param aiMinimum The minimum position
     * @param aiMaximum The maximum position
     * @param adPercent The percentage to replicate
     * @return Returns the solved integer
     */
    public int percentToPosition(int aiMinimum, int aiMaximum, double adPercent) {
        double range = aiMaximum - aiMinimum;
        return (int) Math.round(aiMinimum + (range * adPercent));
    }
}
