package org.firstinspires.ftc.teamcode.IntoTheDeep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Goal;
import org.firstinspires.ftc.teamcode.util.ConfigManager;

import java.io.IOException;

public class GameGoals {
    ConfigManager config = new ConfigManager("TeamCode/src/main/res/raw/robot.properties");
    private DcMotor[] motors;
    private Servo[] servos;
    private Goal[] level1Hang = {
            new Goal(motors, servos, new int[]{500, 29, 0, 1}),
            new Goal(motors, servos, new int[]{0, 0, 0, 1})
    };
    private Goal[] level2Hang = {
    };
    private Goal[] lowerSpecimen = {
    };
    private Goal[] upperSpecimen = {
    };
    private Goal[] lowerBasket = {
    };
    private Goal[] upperBasket = {
    };
    private Goal[] zeroPosition = {
    };

    public GameGoals(DcMotor[] iMotors, Servo[] iServos) throws IOException {
        this.motors = iMotors;
        this.servos = iServos;
    }
}
