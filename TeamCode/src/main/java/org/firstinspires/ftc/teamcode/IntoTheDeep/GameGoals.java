package org.firstinspires.ftc.teamcode.IntoTheDeep;

import android.os.Build;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Goal;
import org.firstinspires.ftc.teamcode.util.ConfigManager;

import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;

public class GameGoals {
    // Array Reference:
    // { Arm_Extend, Arm_PhaseTwo, Arm_Twist, ServoClaw }

    ConfigManager config = new ConfigManager("TeamCode/src/main/res/raw/robot.properties");
    private DcMotor[] motors;
    private Servo[] servos;
    public final Goal[] level1Hang = {
            new Goal("L1 Hang - Raise",motors, servos, new int[]{500, 29, 0, 1}, new double[]{0.8, 0.8, 0.8, 0.8}),
            new Goal("L1 Hang - Lift", motors, servos, new int[]{0, 29, 0, 1}, new double[]{0.8, 0.8, 0.8, 0.8})
    };
    public final Goal[] level2Hang = {
            new Goal("L2 Hang - Raise", motors, servos, new int[]{500, 29, 0, 1}, new double[]{0.8, 0.8, 0.8, 0.8}),
            new Goal("L2 Hang - Lift", motors, servos, new int[]{0, 0, 0, 1}, new double[]{0.8, 0.8, 0.8, 0.8})
    };
    public final Goal[] lowerSpecimen = {
    };
    public final Goal[] upperSpecimen = {
    };
    public final Goal[] lowerBasket = {
    };
    public final Goal[] upperBasket = {
            new Goal("U Basket - Raise",motors, servos, new int[]{1800, 0, 0, 1}, new double[]{0.8, 0.8, 0.8, 0.8})
    };
    public final Goal[] zeroPosition = {
            new Goal("Zero Position",motors, servos, new int[]{0, 0, 0, 1}, new double[]{0.8, 0.8, 0.8, 0.8})
    };

    public GameGoals(DcMotor[] iMotors, Servo[] iServos) throws IOException {
        this.motors = iMotors;
        this.servos = iServos;
    }

    public void executeObjective(Telemetry telemetry, ElapsedTime runtime, Goal[] objective, double PostWaitTime, boolean mbFloor) {
        for (Goal goal : objective) {
            // Reset timer, log that the goal is beginning
            runtime.reset();
            telemetry.addLine("Starting goal sequence");
            telemetry.update();

            // Run Goal
            goal.RunToGoal(mbFloor, 0.8);

            // Keep the script busy for the set pause time
            while(goal.isBusy()) {
                System.out.println(goal.toString());
            }

            // Reset once more to be sure
            runtime.reset();
            while(runtime.seconds() < PostWaitTime) {
                telemetry.addLine("Running goal" + goal.toString());
                telemetry.update();
            };
        }
    }
}
