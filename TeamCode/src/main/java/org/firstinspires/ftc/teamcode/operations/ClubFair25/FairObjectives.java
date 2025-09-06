package org.firstinspires.ftc.teamcode.operations.ClubFair25;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.Collection;
import org.firstinspires.ftc.teamcode.core.Objective;

public class FairObjectives {
    private final DcMotor[] moMotors;
    private final Servo[] moServos;

    // Positions for Reference
    private final int miElbowUp = 860;
    private final int miElbowForward = 450;
    private final int miElbowAngled = 760;
    private final int miWristLeftMax = -140;
    private final int miWristRightMax = 140;
    private final int miExtendMax = -3900;

    public FairObjectives(DcMotor[] aoMotors, Servo[] aoServos) {
        moMotors = aoMotors;
        moServos = aoServos;
    }

    // ========================
    // BEGIN OBJECTIVES & TASKS
    // ========================


    // Say Hi!
    public final Objective foWave(double adArmPower) {
        // Build Tasks Array
        // Each Task takes a 2D array where:
        // - Row 0 → Servo positions
        // - Row 1 → Motor positions
        // - Row 2 → Motor power(s)`
        Collection wave = new Collection();
        wave.add(new double[][]{ {1.0}, {-700, 0, 0}, {adArmPower} }, 4);
        wave.add(new double[][]{ {0}, {-1, 430, 100}, {adArmPower * 0.5} }, 0);
        wave.add(new double[][]{ {1.0}, {-1, 560, -100}, {adArmPower * 0.5} }, 0);
        wave.add(new double[][]{ {0}, {-1, 430, 100}, {adArmPower * 0.5} }, 0);
        wave.add(new double[][]{ {1.0}, {-1, 560, -100}, {adArmPower * 0.5} }, 0);
        wave.add(new double[][]{ {0}, {-1, 430, 100}, {adArmPower * 0.5} }, 0);
        wave.add(new double[][]{ {1.0}, {-1, 560, -100}, {adArmPower * 0.5} }, 0);
        wave.add(new double[][]{ {1.0}, {-1, -1, -1}, {adArmPower} }, 2);
        wave.add(new double[][]{ {1.0}, {0, 0, 0}, {adArmPower} }, 0);

        // Return the array of tasks as an objective object
        return wave.buildObjective(moMotors, moServos);
    }

    public final Objective foHappi(double adArmPower) {
        Collection happi = new Collection();
        happi.add(new double[][]{ {0}, {miExtendMax, -1, -1}, {adArmPower} }, 1);
        happi.add(new double[][]{ {0}, {miExtendMax + 1700, -1, -1}, {adArmPower} }, 0);
        happi.add(new double[][]{ {0}, {miExtendMax, -1, -1}, {adArmPower} }, 0);
        happi.add(new double[][]{ {0}, {miExtendMax + 2900, -1, -1}, {adArmPower} }, 0);
        happi.add(new double[][]{ {0}, {miExtendMax, -1, -1}, {adArmPower} }, 0);
        happi.add(new double[][]{ {0}, {miExtendMax + 2900, -1, -1}, {adArmPower} }, 0);
        happi.add(new double[][]{ {1.0}, {0, 0, 0}, {adArmPower} }, 0);

        return happi.buildObjective(moMotors, moServos);
    }

    // Grab Sample From Floor
    public final Objective foGrabFromFloor(double adArmPower) {
        Collection grabFloor = new Collection();
        grabFloor.add(new double[][] { {0.0}, {0, 0, 0}, {adArmPower} }, 1);
        grabFloor.add(new double[][] { {1.0}, {-1, -1, -1}, {adArmPower} }, 0);

        return grabFloor.buildObjective(moMotors, moServos);
    }

    // Grab Sample From Lower Rung
    public final Objective foGrabFromLowerRung(double adArmPower) {
        Collection grabLowerRung = new Collection();
        grabLowerRung.add(new double[][] { {0.0}, {0, miElbowForward, 0}, {adArmPower} }, 0);
        grabLowerRung.add(new double[][] { {1.0}, {-1, -1, -1}, {adArmPower} }, 1);
        grabLowerRung.add(new double[][] { {1.0}, {-600, -1, -1}, {adArmPower} }, 0);
        grabLowerRung.add(new double[][] { {1.0}, {0, 0, 0}, {adArmPower} }, 0);


        return grabLowerRung.buildObjective(moMotors, moServos);
    }

    public final Objective foHang(double aoArmPower) {
        // Build Tasks Array
        Collection hang = new Collection();
        hang.add(new double[][] { {1}, {-3500, 860, -1}, {aoArmPower} }, 1);
        hang.add(new double[][] { {1}, {0, -1, -1}, {aoArmPower} }, 0);

        return hang.buildObjective(moMotors, moServos);
    }

    // Return to Zero Positions
    public final Objective foZero(double aoArmPower) {
        // Build Tasks Array
        Collection zero = new Collection();
        zero.add(new double[][] { {0.0}, {0, 0, 0}, {aoArmPower} }, 0);

        return zero.buildObjective(moMotors, moServos);
    }

    // ======================
    // END OBJECTIVES & TASKS
    // ======================
}
