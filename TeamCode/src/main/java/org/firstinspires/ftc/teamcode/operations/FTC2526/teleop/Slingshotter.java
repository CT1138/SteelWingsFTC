package org.firstinspires.ftc.teamcode.operations.FTC2526.teleop; // package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.core.util.Mecanum;

import java.io.IOException;
import java.util.TreeMap;

@TeleOp(name="Slingshotter", group="Decode")
public class Slingshotter extends OpMode
{
    // define device classes
    Mecanum moMecanum;
    IMU imu;

    // Auxiliary variables
    private boolean isSequenceRunning = false;
    private boolean lastAState = false;

        // {x, y}
        // X = idle, Y = enabled
    private final double[] flywheelPowers = {0.0, 1};
    private final double[] stopperPositions = {0.6, 0.2};
    private final double[] intakePowers = {0.5, 1};

    // Timer for sequencing
    private ElapsedTime moRuntime;
    private ElapsedTime sequenceTimer;

    // Actuators
    private DcMotorEx moDrive_FrontLeft = null;
    private DcMotorEx moDrive_FrontRight = null;
    private DcMotorEx moDrive_RearLeft = null;
    private DcMotorEx moDrive_RearRight = null;
    private DcMotorEx moAux_Flywheel = null;
    private DcMotorEx moAux_Intake = null;
    private Servo soAux_Stopper = null;

    // throws IOException as some utility classes I wrote require file operations
    public Slingshotter() throws IOException {
    }

    // Prep our motors and servos
    @Override
    public void init() {
        // Actuator setup
        moDrive_FrontLeft = hardwareMap.get(DcMotorEx.class, "FL");
        moDrive_FrontRight = hardwareMap.get(DcMotorEx.class, "FR");
        moDrive_RearLeft = hardwareMap.get(DcMotorEx.class, "RL");
        moDrive_RearRight = hardwareMap.get(DcMotorEx.class, "RR");

        moDrive_FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        moDrive_FrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        moDrive_RearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        moDrive_RearRight.setDirection(DcMotorSimple.Direction.FORWARD);

        moDrive_FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moDrive_FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moDrive_RearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moDrive_RearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        moDrive_FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moDrive_FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moDrive_RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moDrive_RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        moAux_Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        moAux_Flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        soAux_Stopper = hardwareMap.get(Servo.class, "Stopper");

        moAux_Flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //moAux_Flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        moAux_Flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Start timers
        sequenceTimer = new ElapsedTime();
        moRuntime = new ElapsedTime();

        // PID Drive initialization
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDir = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDir = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logoDir, usbDir)));

        moMecanum = new Mecanum(0.45);
    }

    private double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    // Reset our timer
    @Override
    public void start() {
        moRuntime.reset();
    }

    private void operator() {
        Gamepad gpOperator = gamepad2;
        boolean enableFlywheel = gpOperator.left_bumper;
        boolean releaseStopper = gpOperator.right_bumper;

        double flywheelPower = 0;
        double stopperPosition = 0;
        double intakePower = 0;

        flywheelPower = enableFlywheel ? flywheelPowers[1] : flywheelPowers[0];

        boolean canFire = releaseStopper && moAux_Flywheel.getVelocity() > 1200.00;
        stopperPosition = canFire ? stopperPositions[1] : stopperPositions[0];

        if (flywheelPower > 0) {
            gpOperator.rumble(10);
        }

        // --- Apply outputs ---
        moAux_Flywheel.setPower(flywheelPower);
        moAux_Intake.setPower(intakePower);
        soAux_Stopper.setPosition(stopperPosition);
    }

    // Traction Control system
    public double[] tractionControl(double[] powers, double[] velocities, double slipThreshold) {
        // Calculate the average velosity by adding each value and dividing by the number of values
        double avgVel = (velocities[0] + velocities[1] + velocities[2] + velocities[3]) / 4.0;

        // Loop through each value and calculate how much to reduce its power
        for (int i = 0; i < 4; i++) {
            if (avgVel > 50 && velocities[i] > avgVel * slipThreshold) {

                double slipRatio = velocities[i] / avgVel;
                double reduction = Math.min(0.5, (slipRatio - 1.0) * 0.5);
                System.out.println("Traction Control activated: " + slipRatio);
                powers[i] *= (1.0 - reduction);
            }
        }

        return powers;
    }


    private void driver() {
        Gamepad gpDriver = gamepad1;
        // Controls definition
        double mdDrive = gpDriver.left_stick_y;
        double mdStrafe = -gpDriver.left_stick_x;
        double mdTwist = -gpDriver.right_stick_x;
        double mdBrake = gpDriver.right_trigger;
        boolean mbDriveLeft = gpDriver.dpad_left;
        boolean mbDriveRight = gpDriver.dpad_right;
        boolean mbDriveUp = gpDriver.dpad_up;
        boolean mbDriveDown = gpDriver.dpad_down;
        boolean mbTwistLeft = gpDriver.x;
        boolean mbTwistRight = gpDriver.b;

        // Button based controls, use mdBrake to adjust speed
        if (mbDriveLeft) mdStrafe = 1 - mdBrake;
        if (mbDriveRight) mdStrafe = -1 + mdBrake;
        if (mbDriveUp) mdDrive = -1 + mdBrake;
        if (mbDriveDown) mdDrive = 1 - mdBrake;

        if (mbTwistLeft) mdTwist = 1 - mdBrake;
        if (mbTwistRight) mdTwist = -1 + mdBrake;

        // Calculate the target power for all wheels assuming a mecanum drivetrain is in use
        // see teamcode.core.util.Mecanum

        double[] velocities = new double[4];
        velocities[0] = moDrive_FrontLeft.getVelocity();
        velocities[1] = moDrive_FrontRight.getVelocity();
        velocities[2] = moDrive_RearLeft.getVelocity();
        velocities[3] = moDrive_RearRight.getVelocity();

        double[] wheelpower = moMecanum.Calculate(mdDrive, mdStrafe, mdTwist, gamepad1.right_bumper);
        wheelpower = tractionControl(wheelpower, velocities, 1.1);

        moDrive_FrontLeft.setPower(wheelpower[0]);
        moDrive_FrontRight.setPower(wheelpower[1]);
        moDrive_RearLeft.setPower(wheelpower[2]);
        moDrive_RearRight.setPower(wheelpower[3]);
    }

    // Method to store telemetry data
    public void telecom() {
        telemetry.addLine("===================================");
        telemetry.addLine("DriveTrain");
        telemetry.addLine("===================================");
        telemetry.addData("Calculated Heading:", getHeading());
        telemetry.addData("Front Left Power", moDrive_FrontLeft.getPower());
        telemetry.addData("Front Right Power", moDrive_FrontRight.getPower());
        telemetry.addData("Rear Left Power", moDrive_RearLeft.getPower());
        telemetry.addData("Rear Right Power", moDrive_RearRight.getPower());

        telemetry.addData("Rear Left Velocity", moDrive_RearLeft.getVelocity());
        telemetry.addData("Rear Right Velocity", moDrive_RearRight.getVelocity());
        telemetry.addData("Front Left Velocity", moDrive_FrontLeft.getVelocity());
        telemetry.addData("Front Right Velocity", moDrive_FrontRight.getVelocity());

        telemetry.addData("Front Left Position", moDrive_FrontLeft.getCurrentPosition());
        telemetry.addData("Front Right Position", moDrive_FrontRight.getCurrentPosition());
        telemetry.addData("Rear Left Position", moDrive_RearLeft.getCurrentPosition());
        telemetry.addData("Rear Right Position", moDrive_RearRight.getCurrentPosition());
        telemetry.addLine("===================================");
        telemetry.addLine("Auxiliary");
        telemetry.addLine("===================================");
        telemetry.addData("Stopper Position", soAux_Stopper.getPosition());

        // Calculate RPM of the flywheel motor
        double mdFlywheelPower = moAux_Flywheel.getPower();
        double mdFlywheelVelocity = moAux_Flywheel.getVelocity();
        double miFlywheelRPM = (mdFlywheelVelocity / 28) * 60;
        telemetry.addData("Flywheel Power", mdFlywheelPower);
        telemetry.addData("Flywheel Velocity", mdFlywheelVelocity);
        telemetry.addData("Flywheel RPM", miFlywheelRPM);
        telemetry.addData("Intake Power", moAux_Intake.getPower());
        // Update data
        telemetry.update();
    }


    // I like to keep loop() fairly minimal and only reference other methods when possible
    @Override
    public void loop() {
        driver();
        operator();
        telecom();
    }

}
