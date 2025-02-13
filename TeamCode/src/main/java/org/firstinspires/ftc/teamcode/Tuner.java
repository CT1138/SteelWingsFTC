package org.firstinspires.ftc.teamcode; // package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.ConfigManager;
import org.firstinspires.ftc.teamcode.util.MathUtilities;
import org.firstinspires.ftc.teamcode.util.Mecanum;
import org.firstinspires.ftc.teamcode.util.CSVreader;

import java.io.IOException;

@TeleOp(name="Manual Tuner", group="Into-The-Deep")
public class Tuner extends OpMode
{
    private final ElapsedTime runtime = new ElapsedTime();
    Mecanum mecanum = new Mecanum();
    MathUtilities math;
    ConfigManager config;
    ConfigManager devices;
    CSVreader csv;

    // define the motors and whatnot
    private DcMotor Drive_FrontLeft = null;
    private DcMotor Drive_FrontRight = null;
    private DcMotor Drive_RearLeft = null;
    private DcMotor Drive_RearRight = null;
    private DcMotor Arm_Extend = null;
    private DcMotor Arm_PhaseTwo = null;
    private DcMotor Arm_Twist = null;
    private Servo ServoClaw = null;
    private TouchSensor LiftarmStop = null;

    public Tuner() throws IOException {
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

        csv = new CSVreader("TeamCode/src/main/assets/objective_temporary.csv", new DcMotor[]{Drive_FrontLeft, Drive_FrontRight, Drive_RearLeft, Drive_RearRight}, new Servo[]{ServoClaw});
    }

    @Override
    public void start() {
        runtime.reset();
        ServoClaw.setPosition(1);

    }

    private void liftarmMaster() throws IOException {
        // Arm Lift power
        double mdElbowPower = 0;
        mdElbowPower = -gamepad2.right_stick_y;
        if(!gamepad2.left_bumper) mdElbowPower = mdElbowPower * 0.5;
        Arm_PhaseTwo.setPower(mdElbowPower);

        // Arm extension
        double mdExtendPower = gamepad2.left_stick_y;
        if (LiftarmStop.isPressed() && gamepad2.left_stick_y > 0) {
            mdExtendPower = 0;
        }
        Arm_Extend.setPower(mdExtendPower);

        // Arm Twist using power and position limits

        Arm_Twist.setPower(gamepad2.left_stick_x / 10);

        if (gamepad2.dpad_up || gamepad2.dpad_down) {
            if (gamepad2.dpad_up) {
                ServoClaw.setPosition(-1);
            }

            if (gamepad2.dpad_down) {
                ServoClaw.setPosition(1);
            }
        }

        if (gamepad2.a && runtime.seconds() > 2) {
            float mfPower = gamepad2.left_trigger;

            int miExtendPosition = Arm_Extend.getCurrentPosition();
            int miPhaseTwoPosition = Arm_PhaseTwo.getCurrentPosition();
            int miTwistPosition = Arm_Twist.getCurrentPosition();

            int[] miPositions = {miExtendPosition, miPhaseTwoPosition, miTwistPosition};
            double[] mdPowers = {mfPower, mfPower, mfPower};

            csv.writeTempObjective(miPositions, mdPowers);

            runtime.reset();
        }

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
    }

    @Override
    public void loop() {
        try {
            liftarmMaster();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        drivetrainMaster();
        telemetry.update();
    }

}
