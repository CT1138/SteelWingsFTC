package org.firstinspires.ftc.teamcode.operations.FTC2526.teleop; // package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.core.util.Mecanum;

import java.io.IOException;

@TeleOp(name="Slingshotter", group="Decode")
public class Slingshotter extends OpMode
{
    // CONTROLS:
    private final Gamepad gpDriver = gamepad1;
    private Gamepad gpDriver_previous = new Gamepad();
    private final Gamepad gpOperator = gamepad2;
    private Gamepad gpOperator_previous = new Gamepad();

        private final double    CONTROL_DRIVE = gpDriver.right_stick_y;
        private final int       CONTROL_STRAFE_LEFT = gpDriver.left_bumper ? 1 : 0;
        private final int       CONTROL_STRAFE_RIGHT = gpDriver.right_bumper ? 1 : 0;
        private final double    CONTROL_STRAFE = -gpDriver.left_stick_x + (CONTROL_STRAFE_LEFT - CONTROL_STRAFE_RIGHT);
        private final double    CONTROL_TWIST = -gpDriver.right_stick_x;
        private final double    CONTROL_GAS = gpDriver.right_trigger;
        private final double    CONTROL_BRAKE = gpDriver.left_trigger;
        private final boolean   CONTROL_FLYWHEEL = gpOperator.left_bumper || gpOperator.a;
        private final boolean   CONTROL_INTAKE = gpOperator.dpad_up;
        private final boolean   CONTROL_STOPPER = gpOperator.right_bumper || gpOperator.y;

    // SETTINGS
        private final double    DRIVE_MAX_SPEED = 0.4;
        private final int       DRIVE_MAX_RPM = 5500;
        private final double    DRIVE_SLIP_THRESHOLD = 1.2;
        private final int       FLYWHEEL_MAX_RPM = 4500;
        private final int       INTAKE_MAX_RPM = 100;

        private final int       DRIVE_COUNTS_PER_REVOLUTION = 28;
        private final int       FLYWHEEL_COUNTS_PER_REVOLUTION = 28;
        private final int       INTAKE_COUNTS_PER_REVOLUTION = 288;
        RevHubOrientationOnRobot.LogoFacingDirection
                CONTROL_LOGO_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection
                CONTROL_USB_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.UP;


    // SYSTEM VARIABLES -- LEAVE BE
    private boolean flywheelActive;

    // FINALS
    private final double[] flywheelVelocities = {
            0,
            rpmToVelocity(
                    FLYWHEEL_MAX_RPM,
                    FLYWHEEL_COUNTS_PER_REVOLUTION
            )
    };
    private final double[] intakeVelocities = {
            0,
            rpmToVelocity(
                    INTAKE_MAX_RPM,
                    INTAKE_COUNTS_PER_REVOLUTION
            )
    };
    private final double[] stopperPositions = {
            0.5,
            0
    };

    // Objects
    private Mecanum mecanum;
    private IMU imu;
    private ElapsedTime elapsedTime;
    private ElapsedTime rampTime;
    private DcMotorEx driveFL = null;
    private DcMotorEx driveFR = null;
    private DcMotorEx driveRL = null;
    private DcMotorEx driveRR = null;
    private DcMotorEx auxFlywheel = null;
    private DcMotorEx auxIntake = null;
    private Servo auxStopper = null;
    private TouchSensor loadSensor = null;

    // throws IOException as some utility classes I wrote require file operations
    public Slingshotter() throws IOException {
    }

    private double rpmToVelocity(double RPM, int countsPerRevolution) {
        return ( RPM / 60 ) * countsPerRevolution;
    }

    private double velocityToRPM(double Velocity, int countsPerRevolution) {
        return ( Velocity / countsPerRevolution ) * 60;
    }

    // Prep our motors and servos
    @Override
    public void init() {
        // Actuator setup
        DcMotorEx[] driveTrainMotors = {driveFL, driveFR, driveRL, driveRR};
        driveFL = hardwareMap.get(DcMotorEx.class, "FL");
        driveFR = hardwareMap.get(DcMotorEx.class, "FR");
        driveRL = hardwareMap.get(DcMotorEx.class, "RL");
        driveRR = hardwareMap.get(DcMotorEx.class, "RR");

        for (DcMotorEx motor : driveTrainMotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        driveFL.setDirection(DcMotorSimple.Direction.REVERSE);
        driveFR.setDirection(DcMotorSimple.Direction.FORWARD);
        driveRL.setDirection(DcMotorSimple.Direction.REVERSE);
        driveRR.setDirection(DcMotorSimple.Direction.FORWARD);

        auxIntake = hardwareMap.get(DcMotorEx.class, "Intake");
        auxIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        auxIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        auxFlywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        auxFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        auxFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        loadSensor = hardwareMap.get(TouchSensor.class, "LoadSensor");
        auxStopper = hardwareMap.get(Servo.class, "Stopper");
        // Start timers
        elapsedTime = new ElapsedTime();
        rampTime = new ElapsedTime();

        // PID Drive initialization
        imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(CONTROL_LOGO_DIRECTION, CONTROL_USB_DIRECTION)));

        mecanum = new Mecanum(0.45);
    }

    // Reset our timer
    @Override
    public void start() {
        elapsedTime.reset();
        rampTime.reset();
    }

    private void operator() {
        // Initial Powers
        double flywheelVelocity = 0;
        double stopperPosition = 0;
        double intakeVelocity = 0;

        // Flywheel
        flywheelVelocity = CONTROL_FLYWHEEL ? flywheelVelocities[1] : flywheelVelocities[0];
        flywheelActive = flywheelVelocity > 0;

        // Intake
        intakeVelocity = (CONTROL_INTAKE || loadSensor.isPressed()) ? intakeVelocities[1] : intakeVelocities[0];
        if (flywheelActive) intakeVelocity = 0;

        // Stopper
        stopperPosition = CONTROL_STOPPER ? stopperPositions[1] : stopperPositions[0];


        if (flywheelVelocity > 0) {
            gpOperator.rumble(250);
        }

        // --- Apply outputs ---
        auxFlywheel.setVelocity(flywheelVelocity);
        auxIntake.setVelocity(intakeVelocity);
        auxStopper.setPosition(stopperPosition);
    }

    private double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    // Traction Control system
    private double[] tractionControl(double[] powers, double[] velocities, double slipThreshold) {
        // Calculate the average velocity by adding each value and dividing by the number of values
        double avgVelocity = (velocities[0] + velocities[1] + velocities[2] + velocities[3]) / 4.0;

        // Loop through each value and calculate how much to reduce its power
        for (int i = 0; i < 4; i++) {
            if (avgVelocity > 50 && velocities[i] > avgVelocity * slipThreshold) {
                gpDriver.rumble(250);

                double slipRatio = velocities[i] / avgVelocity;
                double reduction = Math.min(0.5, (slipRatio - 1.0) * 0.5);

                powers[i] *= (1.0 - reduction);
            }
        }

        telemetry.addData("Traction Control Factors", powers);
        return powers;
    }

    private void driver() {
        double drive = CONTROL_DRIVE;
        double strafe = CONTROL_STRAFE;
        double twist = CONTROL_TWIST;

        double[] velocities = new double[4];
        velocities[0] = driveFL.getVelocity();
        velocities[1] = driveFR.getVelocity();
        velocities[2] = driveRL.getVelocity();
        velocities[3] = driveRR.getVelocity();

        double modifier = DRIVE_MAX_SPEED + (CONTROL_GAS - CONTROL_BRAKE) * 0.5;
        modifier = Math.max(0, Math.min(1, modifier));
        drive *= modifier;
        strafe *= modifier;
        twist *= modifier;

        double[] wheelPower = mecanum.Calculate(drive, strafe, twist);
        wheelPower = tractionControl(wheelPower, velocities, DRIVE_SLIP_THRESHOLD);

        double maxVelocity = rpmToVelocity(DRIVE_MAX_RPM, DRIVE_COUNTS_PER_REVOLUTION);
        driveFL.setVelocity(wheelPower[0] * maxVelocity);
        driveFR.setVelocity(wheelPower[1] * maxVelocity);
        driveRL.setVelocity(wheelPower[2] * maxVelocity);
        driveRR.setVelocity(wheelPower[3] * maxVelocity);
    }

    // Method to store telemetry data
    private void telecom() {
        telemetry.addLine("===================================");
        telemetry.addLine("DriveTrain");
        telemetry.addLine("===================================");
        telemetry.addData("Calculated Heading:", getHeading());
        telemetry.addLine("");
        telemetry.addData("Front Left Power", driveFL.getPower());
        telemetry.addData("Front Left Velocity", driveFL.getVelocity());
        telemetry.addData("Front Left RPM", velocityToRPM(driveFL.getVelocity(), DRIVE_COUNTS_PER_REVOLUTION));
        telemetry.addData("Front Left Position", driveFL.getCurrentPosition());
        telemetry.addLine("");
        telemetry.addData("Front Right Power", driveFR.getPower());
        telemetry.addData("Front Right Velocity", driveFR.getVelocity());
        telemetry.addData("Front Right RPM", velocityToRPM(driveFR.getVelocity(), DRIVE_COUNTS_PER_REVOLUTION));
        telemetry.addData("Front Right Position", driveFR.getCurrentPosition());
        telemetry.addLine("");
        telemetry.addData("Rear Left Power", driveRL.getPower());
        telemetry.addData("Rear Left Velocity", driveRL.getVelocity());
        telemetry.addData("Rear Left RPM", velocityToRPM(driveRL.getVelocity(), DRIVE_COUNTS_PER_REVOLUTION));
        telemetry.addData("Rear Left Position", driveRL.getCurrentPosition());
        telemetry.addLine("");
        telemetry.addData("Rear Right Power", driveRR.getPower());
        telemetry.addData("Rear Right Velocity", driveRR.getVelocity());
        telemetry.addData("Rear Right RPM", velocityToRPM(driveRR.getVelocity(), DRIVE_COUNTS_PER_REVOLUTION));
        telemetry.addData("Rear Right Position", driveRR.getCurrentPosition());
        telemetry.addLine("===================================");
        telemetry.addLine("Auxiliary");
        telemetry.addLine("===================================");
        telemetry.addData("Stopper Position", auxStopper.getPosition());
        telemetry.addData("Flywheel Power", auxFlywheel.getPower());
        telemetry.addData("Flywheel Velocity", auxFlywheel.getVelocity());
        telemetry.addData("Flywheel RPM", velocityToRPM(auxFlywheel.getVelocity(), FLYWHEEL_COUNTS_PER_REVOLUTION));
        telemetry.addLine("");
        telemetry.addData("Intake Power", auxIntake.getPower());
        telemetry.addData("Intake Velocity", auxIntake.getVelocity());
        telemetry.addData("Intake RPM", velocityToRPM(auxIntake.getVelocity(), INTAKE_COUNTS_PER_REVOLUTION));
        // Update data
        telemetry.update();
    }


    // I like to keep loop() fairly minimal and only reference other methods when possible
    @Override
    public void loop() {
        driver();
        operator();
        telecom();

        gpDriver_previous.copy(gpDriver);
        gpOperator_previous.copy(gpOperator);
    }

}
