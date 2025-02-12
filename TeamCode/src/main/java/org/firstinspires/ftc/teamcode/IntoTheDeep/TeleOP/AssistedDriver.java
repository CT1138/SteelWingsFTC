package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOP; // package org.firstinspires.ftc.robotcontroller.external.samples;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.IntoTheDeep.GameGoals;
import org.firstinspires.ftc.teamcode.util.ConfigManager;
import org.firstinspires.ftc.teamcode.util.Goal;
import org.firstinspires.ftc.teamcode.util.MathUtilities;
import org.firstinspires.ftc.teamcode.util.Mecanum;
import org.firstinspires.ftc.teamcode.util.CSVreader;

import java.io.IOException;

@TeleOp(name="Assisted Driver", group="Into-The-Deep")
public class AssistedDriver extends OpMode
{
    // define the motors and whatnot
    Mecanum mecanum = new Mecanum();
    MathUtilities math = new MathUtilities();
    ConfigManager config = new ConfigManager("TeamCode/src/main/res/raw/robot.properties");
    ConfigManager devices = new ConfigManager("TeamCode/src/main/res/raw/devices.properties");
    private DcMotor[] motors;
    private Servo[] servos;
    private GameGoals gameGoals;
    private Boolean DriverIsBusy = false;
    private boolean isGoalRunning = false;
    private Goal currentGoal = null;
    private int currentGoalIndex = 0;
    private Goal[] currentGoals = null;
    private final int ARM_TWIST_MIN   = config.getInt("ARM_TWIST_MIN"); //-140; // Equivalent to -180 degrees
    private final int ARM_TWIST_MAX   = config.getInt("ARM_TWIST_MAX"); //140;  // Equivalent to 180 degrees
    private int armTwistPositionIndex = 0;
    private int armTwistStartingPosition     = config.getInt("ARM_TWIST_START");

    private final int ARM_EXTEND_MIN = config.getInt("ARM_EXTEND_MIN");
    private final int ARM_EXTEND_MAX = config.getInt("ARM_EXTEND_MAX");
    private int armExtendStartingPosition   = config.getInt("ARM_EXTEND_START");

    private final int ARM_ELBOW_MIN = config.getInt("ARM_ELBOW_MIN");
    private final int ARM_ELBOW_MAX = config.getInt("ARM_ELBOW_MAX");
    private int armElbowStartingPosition = config.getInt("ARM_ELBOW_START");

    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor Drive_FrontLeft = null;
    private DcMotor Drive_FrontRight = null;
    private DcMotor Drive_RearLeft = null;
    private DcMotor Drive_RearRight = null;
    private DcMotor Arm_Extend = null;
    private DcMotor Arm_PhaseTwo = null;

    private DcMotor Arm_Twist = null;

    private Servo ServoClaw = null;
    private TouchSensor LiftarmStop = null;

    public AssistedDriver() throws IOException {
    }

    @Override
    public void init() {
        // Single execution on INIT
        Drive_FrontLeft  = hardwareMap.get(DcMotor.class, "Drive_FrontLeft");
        Drive_FrontRight = hardwareMap.get(DcMotor.class, "Drive_FrontRight");
        Drive_RearLeft   = hardwareMap.get(DcMotor.class, "Drive_RearLeft");
        Drive_RearRight  = hardwareMap.get(DcMotor.class, "Drive_RearRight");

        Arm_Extend = hardwareMap.get(DcMotor.class, "Arm_Extend");
        Arm_PhaseTwo = hardwareMap.get(DcMotor.class, "Arm_PhaseTwo");
        Arm_Twist = hardwareMap.get(DcMotor.class, "Arm_Twist");

        ServoClaw = hardwareMap.get(Servo.class, "Servo_Claw");
        LiftarmStop = hardwareMap.get(TouchSensor.class, "TouchSensor");

        Drive_FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Drive_FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Drive_RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Drive_RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Arm_Extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm_PhaseTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Arm_Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Drive_FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Drive_FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Drive_RearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Drive_RearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors = new DcMotor[]{Arm_Extend, Arm_PhaseTwo, Arm_Twist};
        servos = new Servo[]{ServoClaw};
        try {
            gameGoals = new GameGoals(motors, servos);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void start() {
        runtime.reset();
        ServoClaw.setPosition(1);
        armExtendStartingPosition = Arm_Extend.getCurrentPosition();
        armElbowStartingPosition = Arm_PhaseTwo.getCurrentPosition();
    }

    private void liftarmMaster() {
        boolean mbFloor = LiftarmStop.isPressed();

        if (gamepad2.left_bumper && gamepad2.right_bumper) {
            Arm_Extend.setPower(0);
            Arm_Twist.setPower(0);
            Arm_PhaseTwo.setPower(0);
            ServoClaw.setPosition(0.5);
        }

        if (!DriverIsBusy && gamepad2.a && !isGoalRunning) {
            // Start a new goal sequence
            /*
            Goal[] currentGoals =
                {
                    new Goal("Zero Position", motors, servos, new int[]{0, 0, 0, 1}, new double[]{0.8, 0.8, 0.8, 0.8}),
                    new Goal("L1 Hang - Raise",motors, servos, new int[]{-4100, 430, 5, 1}, new double[]{0.8, 0.8, 0.8, 0.8}),
                    new Goal("Zero Position",motors, servos, new int[]{0, 0, 0, 1}, new double[]{0.8, 0.8, 0.8, 0.8})
                }; //gameGoals.level1Hang;
             */
            CSVreader objective_template = new CSVreader("TeamCode/src/main/assets/objective_template.csv", motors, servos);
            Goal[] currentGoals = objective_template.readCSV();
            gameGoals.executeObjective(telemetry, runtime, currentGoals, 2, mbFloor);
        }

        if (!DriverIsBusy && gamepad2.y && !isGoalRunning) {
            System.out.println("Starting goal sequence");
            // Start a new goal sequence
            Goal[] currentGoals =
                    {
                            new Goal("Zero Position", motors, servos, new int[]{0, 0, 0, 1}, new double[]{0.8, 0.8, 0.8, 0.8}),
                    }; //gameGoals.level1Hang;

            gameGoals.executeObjective(telemetry, runtime, currentGoals, 2, mbFloor);
        }

        /*
        if (isGoalRunning && !currentGoal.isBusy()) {
            // Check if the current goal is still running and if Current goal is finished, move to the next or reset
            currentGoalIndex++;
            if (currentGoalIndex < currentGoals.length) {
                currentGoal = currentGoals[currentGoalIndex];
                currentGoal.RunToGoal(0.8, 2);
            } else {
                // All goals are finished
                isGoalRunning = false;
                currentGoal = null;
                currentGoals = null;
            }
        }
        */

        telemetry.addLine("===================================");
        telemetry.addLine("Arm Positions");
        telemetry.addLine("===================================");
        telemetry.addData("Arm Extend Position", Arm_Extend.getCurrentPosition());
        telemetry.addData("Arm Elbow Position", Arm_PhaseTwo.getCurrentPosition());
        telemetry.addData("Arm Twist Position", Arm_Twist.getCurrentPosition());
        telemetry.addData("Arm Claw Position", ServoClaw.getPosition());
        telemetry.addLine("===================================");
    }

    private void drivetrainMaster() {
        double drive = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double twist = gamepad1.right_stick_x;

        boolean slow = gamepad1.right_bumper;
        boolean slower = gamepad1.left_bumper;

        double modifier = gamepad1.right_trigger;

        if (gamepad1.dpad_left) strafe = 1 - modifier;
        if (gamepad1.dpad_right) strafe = -1 + modifier;
        if (gamepad1.dpad_up) drive = -1 + modifier;
        if (gamepad1.dpad_down) drive = 1 - modifier;

        if (gamepad1.x) twist = -1 + modifier;
        if (gamepad1.b) twist = 1 - modifier;

        if (slower) {
            drive = drive * 0.5;
            strafe = strafe * 0.5;
            twist = strafe * 0.5;
        }

        if (!slow) {
            drive = drive * 0.75;
            strafe = strafe * 0.75;
            twist = twist * 0.75;
        }

        double[] wheelpower = mecanum.calculate(drive, strafe, -twist, gamepad2.right_bumper);

        Drive_FrontLeft.setPower(wheelpower[0]);
        Drive_FrontRight.setPower(wheelpower[1]);
        Drive_RearLeft.setPower(wheelpower[2]);
        Drive_RearRight.setPower(wheelpower[3]);

        telemetry.addLine("===================================");
        telemetry.addLine("DriveTrain");
        telemetry.addLine("===================================");
        telemetry.addData("Front Left Power", Drive_FrontLeft.getPower());
        telemetry.addData("Front Right Power", Drive_FrontRight.getPower());
        telemetry.addData("Rear Left Power", Drive_RearLeft.getPower());
        telemetry.addData("Rear Right Power", Drive_RearRight.getPower());

        telemetry.addData("Front Left Position", Drive_FrontLeft.getCurrentPosition());
        telemetry.addData("Front Right Position", Drive_FrontRight.getCurrentPosition());
        telemetry.addData("Rear Left Position", Drive_RearLeft.getCurrentPosition());
        telemetry.addData("Rear Right Position", Drive_RearRight.getCurrentPosition());
        telemetry.addLine("===================================");
    }

    @Override
    public void loop() {
        liftarmMaster();
        drivetrainMaster();
        telemetry.update();
    }

}
