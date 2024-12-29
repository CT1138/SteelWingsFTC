package org.firstinspires.ftc.teamcode.IntoTheDeep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Goal;
import org.firstinspires.ftc.teamcode.util.ConfigManager;

import java.io.IOException;

public class GameGoals {
    // Array Reference:
    // { Arm_Extend, Arm_PhaseTwo, Arm_Twist, ServoClaw }
    ConfigManager config = new ConfigManager("TeamCode/src/main/res/raw/robot.properties");
    private DcMotor[] motors;
    private Servo[] servos;
    private final Goal[] level1Hang = {
            new Goal(motors, servos, new int[]{500, 29, 0, 1}),
            new Goal(motors, servos, new int[]{0, 29, 0, 1})
    };
    private final Goal[] level2Hang = {
            new Goal(motors, servos, new int[]{500, 29, 0, 1}),
            new Goal(motors, servos, new int[]{0, 0, 0, 1})
    };
    private final Goal[] lowerSpecimen = {
    };
    private final Goal[] upperSpecimen = {
    };
    private final Goal[] lowerBasket = {
    };
    private final Goal[] upperBasket = {
            new Goal(motors, servos, new int[]{1800, 0, 0, 1})
    };
    private final Goal[] zeroPosition = {
            new Goal(motors, servos, new int[]{0, 0, 0, 1})
    };

    public Goal[] hang(int level) {
        switch(level)
        {
            case 1:
                return level1Hang;
            case 2:
                return level2Hang;
            default:
                return new Goal[0];
        }
    }

    public Goal[] specimen(int level) {
        switch(level)
        {
            case 1:
                return lowerSpecimen;
            case 2:
                return upperSpecimen;
            default:
                return new Goal[0];
        }
    }

    public Goal[] basket(int level) {
        switch(level)
        {
            case 1:
                return lowerBasket;
            case 2:
                return upperBasket;
            default:
                return new Goal[0];
        }
    }

    public Goal[] zero() {
        return zeroPosition;
    }

    public GameGoals(DcMotor[] iMotors, Servo[] iServos) throws IOException {
        this.motors = iMotors;
        this.servos = iServos;
    }
}
