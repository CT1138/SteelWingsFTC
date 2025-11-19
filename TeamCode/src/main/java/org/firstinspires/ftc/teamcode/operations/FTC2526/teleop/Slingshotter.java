package org.firstinspires.ftc.teamcode.operations.FTC2526.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.core.util.Mecanum;

import java.io.IOException;

@TeleOp(name="Slingshotter", group="Decode")
public class Slingshotter extends OpMode
{
    //============================================================================================
    // BEGIN VARIABLES
    // EDIT CONTROLS IN loop()
    //============================================================================================
    // SETTINGS
        // PIDF Settings:
            // (P)roportional, (I)ntegral, (D)erivative, and (F)loat
            // P*: how strongly the controller reacts to the current error
            // I*: how strongly the controller reacts to the accumulation of past error (helps eliminate steady-state error)
            // D*: how strongly the controller reacts to the rate of change of the error (helps reduce overshoot and oscillation)
            // F:  provides a baseline output to reach a target speed or position without error
        private final PIDFCoefficients DRIVE_PIDF = new PIDFCoefficients(10.0, 3.0, 0.0, 0.0);
        private final PIDFCoefficients FLYWHEEL_PIDF = new PIDFCoefficients(10.0, 3.0, 0.0, 0.0);
        private final PIDFCoefficients INTAKE_PIDF = new PIDFCoefficients(10.0, 3.0, 0.0, 0.0);
        // END PIDF SETTINGS

        // CONSTANTS
        private final double    DEADZONE_TRIGGER_BUTTON = 0.5;
        private final double    DEADZONE_STICK = 0.05;
        private final double    OPERATOR_DRIVE_MODIFIER = 0.5;
        private final double    DRIVE_BASE_SPEED = 0.8;
        private final double    TWIST_BASE_SPEED = 0.5;
        private final double    STRAFE_BASE_SPEED = 0.5;

        private final int       DRIVE_MAX_RPM = 5500;
        private final double    DRIVE_SLIP_THRESHOLD = 1.2;
        private final int       FLYWHEEL_MAX_RPM = 5000;
        private final int       INTAKE_MAX_RPM = 5500;

        private final double    STOPPER_OPEN_POSITION = 0.5;
        private final double    STOPPER_CLOSED_POSITION = 0.8;

        private final double    LOADER_OPEN_POSITION = 0.2;
        private final double    LOADER_CLOSED_POSITION = 0.45;

        private final int       DRIVE_COUNTS_PER_REVOLUTION = 28;
        private final int       FLYWHEEL_COUNTS_PER_REVOLUTION = 28;
        private final int       INTAKE_COUNTS_PER_REVOLUTION = 28;

        private final LogoFacingDirection CONTROL_LOGO_DIRECTION = LogoFacingDirection.RIGHT;
        private final UsbFacingDirection CONTROL_USB_DIRECTION = UsbFacingDirection.UP;
        // END CONSTANTS

    // SYSTEM VARIABLES -- LEAVE BE
    private Gamepad gpDriver;
    private Gamepad gpDriver_previous;
    private Gamepad gpOperator;
    private Gamepad gpOperator_previous;

    private double    CONTROL_DRIVE;
    private double    CONTROL_STRAFE;
    private double    CONTROL_TWIST;
    private boolean   CONTROL_HALF_SPEED;
    private boolean   CONTROL_FULL_SPEED;
    private double    CONTROL_GAS;
    private double    CONTROL_BRAKE;
    private boolean   CONTROL_DPAD_STRAFE_LEFT;
    private boolean   CONTROL_DPAD_STRAFE_RIGHT;

    private boolean   CONTROL_FLYWHEEL;
    private boolean   CONTROL_INTAKE;
    private double    CONTROL_INTAKE_VARIABLE;
    private boolean   CONTROL_STOPPER;
    private boolean   CONTROL_LOADER;
    private double    CONTROL_OPERATOR_DRIVE;
    private double    CONTROL_OPERATOR_TWIST;

    private final double[] flywheelVelocities = { 0, this.rpmToVelocity(FLYWHEEL_MAX_RPM, FLYWHEEL_COUNTS_PER_REVOLUTION) };
    private final double[] intakeVelocities = { 0, this.rpmToVelocity(INTAKE_MAX_RPM, INTAKE_COUNTS_PER_REVOLUTION) };
    private final double[] stopperPositions = { STOPPER_CLOSED_POSITION, STOPPER_OPEN_POSITION };
    private final double[] loaderPositions = {LOADER_CLOSED_POSITION, LOADER_OPEN_POSITION};

    private Mecanum         mecanum;
    private IMU             imu;
    private ElapsedTime     elapsedTime;
    private ElapsedTime     rampTime;
    private DcMotorEx       driveFL = null;
    private DcMotorEx       driveFR = null;
    private DcMotorEx       driveRL = null;
    private DcMotorEx       driveRR = null;
    private DcMotorEx       auxFlywheel = null;
    private DcMotorEx       auxIntake = null;
    private Servo           auxStopper = null;
    private Servo           auxLoader   = null;

    //============================================================================================
    // END VARIABLES
    //============================================================================================

    // Throws IOException as some utility classes I wrote require file operations
    public Slingshotter() throws IOException {
    }

    // =============================================================================================
    // HELPER METHODS
    // =============================================================================================
    private double rpmToVelocity(double RPM, int countsPerRevolution) {
        return ( RPM / 60 ) * countsPerRevolution;
    }

    private double velocityToRPM(double Velocity, int countsPerRevolution) {
        return ( Velocity / countsPerRevolution ) * 60;
    }
    private double deadzone(double input, double threshold) {
        return Math.abs(input) < threshold ? 0 : input;
    }
    // =============================================================================================
    // END HELPER METHODS
    // =============================================================================================

    // Prep our motors and servos
    @Override
    public void init() {
        // Actuator setup
        driveFL = hardwareMap.get(DcMotorEx.class, "FL");
        driveFR = hardwareMap.get(DcMotorEx.class, "FR");
        driveRL = hardwareMap.get(DcMotorEx.class, "RL");
        driveRR = hardwareMap.get(DcMotorEx.class, "RR");

        DcMotorEx[] driveTrainMotors = {driveFL, driveFR, driveRL, driveRR};
        for (DcMotorEx motor : driveTrainMotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DRIVE_PIDF);
        }

        driveFL.setDirection(DcMotorSimple.Direction.FORWARD);
        driveFR.setDirection(DcMotorSimple.Direction.REVERSE);
        driveRL.setDirection(DcMotorSimple.Direction.FORWARD);
        driveRR.setDirection(DcMotorSimple.Direction.REVERSE);

        auxIntake = hardwareMap.get(DcMotorEx.class, "Intake");
        auxIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        auxIntake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, INTAKE_PIDF);
        auxIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        auxIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        auxFlywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        auxFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        auxFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, FLYWHEEL_PIDF);
        auxFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        auxStopper = hardwareMap.get(Servo.class, "Stopper");
        auxLoader  = hardwareMap.get(Servo.class, "Loader");

        // IMU Initialization
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(CONTROL_LOGO_DIRECTION, CONTROL_USB_DIRECTION)));

        // Drivetrain handlers
        mecanum = new Mecanum();

        // Start timers
        elapsedTime = new ElapsedTime();
        rampTime = new ElapsedTime();
    }

    // Reset our timers
    @Override
    public void start() {
        elapsedTime.reset();
        rampTime.reset();
    }

    // =============================================================================================
    // OPERATOR METHODS
    // =============================================================================================
    private void operator() {
        // Initial Powers
        double flywheelVelocity = 0;
        double stopperPosition = 0;
        double loaderPosition = 0;
        double intakeVelocity = 0;

        // Flywheel
        flywheelVelocity = CONTROL_FLYWHEEL ? flywheelVelocities[1] : flywheelVelocities[0];
        // Intake
        if (Math.abs(CONTROL_INTAKE_VARIABLE) > 0.1) {
            intakeVelocity = (intakeVelocities[1] * -CONTROL_INTAKE_VARIABLE) * 0.5;
        }

        if (CONTROL_FLYWHEEL && auxFlywheel.getVelocity() >= 2000 || CONTROL_INTAKE) {
            intakeVelocity = intakeVelocities[1];
        }

        // Servos
        stopperPosition = CONTROL_STOPPER ? stopperPositions[1] : stopperPositions[0];
        loaderPosition = CONTROL_LOADER ? loaderPositions[1] : loaderPositions[0];

        // Feedback
        if (flywheelVelocity > 0) {
            gpOperator.rumble(250);
        }

        // Apply outputs
        auxFlywheel.setVelocity(flywheelVelocity);
        auxIntake.setVelocity(intakeVelocity);
        auxStopper.setPosition(stopperPosition);
        auxLoader.setPosition(loaderPosition);
    }
    // =============================================================================================
    // END OPERATOR METHODS
    // =============================================================================================

    // =============================================================================================
    // DRIVER METHODS
    // =============================================================================================
    private double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    private double[] tractionControl(double[] powers, double slipThreshold) {
        double[] velocities = new double[4];
        velocities[0] = driveFL.getVelocity();
        velocities[1] = driveFR.getVelocity();
        velocities[2] = driveRL.getVelocity();
        velocities[3] = driveRR.getVelocity();

        // Calculate the average velocity by adding each value and dividing by the number of values
        double avgVelocity = (velocities[0] + velocities[1] + velocities[2] + velocities[3]) / 4.0;

        // Loop through each value and calculate how much to reduce its power
        for (int i = 0; i < 4; i++) {
            if (avgVelocity > 50 && velocities[i] > avgVelocity * slipThreshold) {
                double slipRatio = velocities[i] / avgVelocity;
                double reduction = Math.min(0.5, (slipRatio - 1.0) * 0.5);

                powers[i] *= (1.0 - reduction);
            }
        }

        telemetry.addData("Traction Control Factors", powers);
        return powers;
    }


    private double driveModifier(double gas, double brake, double base) {
        double modifier = base + (gas * (1 - base)) - (brake * base);

        if (CONTROL_FULL_SPEED) return 1;
        if (CONTROL_HALF_SPEED) return 0.5;
        return Math.max(0, Math.min(1, modifier));
    }

    private void driver() {
        double drive = CONTROL_DRIVE;
        double strafe = CONTROL_STRAFE;
        double twist = CONTROL_TWIST;

        drive *= this.driveModifier(CONTROL_GAS, CONTROL_BRAKE, DRIVE_BASE_SPEED);
        strafe *= this.driveModifier(CONTROL_GAS, CONTROL_BRAKE, STRAFE_BASE_SPEED);
        twist *= this.driveModifier(CONTROL_GAS, CONTROL_BRAKE, TWIST_BASE_SPEED);

        double[] wheelPower = this.tractionControl(mecanum.Calculate(drive, strafe, twist), DRIVE_SLIP_THRESHOLD);

        double maxVelocity = this.rpmToVelocity(DRIVE_MAX_RPM, DRIVE_COUNTS_PER_REVOLUTION);
        driveFL.setVelocity(wheelPower[0] * maxVelocity);
        driveFR.setVelocity(wheelPower[1] * maxVelocity);
        driveRL.setVelocity(wheelPower[2] * maxVelocity);
        driveRR.setVelocity(wheelPower[3] * maxVelocity);
    }
    // =============================================================================================
    // END DRIVER METHODS
    // =============================================================================================


    // =============================================================================================
    // TELEMETRY
    // =============================================================================================
    private void printMotorTelemetry(String name, DcMotorEx motor, DcMotor.RunMode runMode, int counts) {
        PIDFCoefficients pidf = driveFL.getPIDFCoefficients(runMode);
        String pidfString = String.format("P: %.2f I: %.2f D: %.2f F: %.2f", pidf.p, pidf.i, pidf.d, pidf.f);

        telemetry.addLine("========================");
        telemetry.addLine(name);
        telemetry.addData("Power", motor.getPower());
        telemetry.addData("Current (milliamps)", motor.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Velocity", motor.getVelocity());
        telemetry.addData("RPM", this.velocityToRPM(motor.getVelocity(), counts));
        telemetry.addData("Position", motor.getCurrentPosition());
        telemetry.addLine(pidfString);
        telemetry.addLine("========================");
    }
    private void telecom() {
        telemetry.addData("Calculated Heading:", this.getHeading());
        this.printMotorTelemetry("Front Left", driveFL, DcMotor.RunMode.RUN_USING_ENCODER, DRIVE_COUNTS_PER_REVOLUTION);
        this.printMotorTelemetry("Front Right", driveFR, DcMotor.RunMode.RUN_USING_ENCODER, DRIVE_COUNTS_PER_REVOLUTION);
        this.printMotorTelemetry("Rear Left", driveRL, DcMotor.RunMode.RUN_USING_ENCODER, DRIVE_COUNTS_PER_REVOLUTION);
        this.printMotorTelemetry("Rear Right", driveRR, DcMotor.RunMode.RUN_USING_ENCODER, DRIVE_COUNTS_PER_REVOLUTION);
        telemetry.addData("Stopper Position", auxStopper.getPosition());
        telemetry.addData("Loader Position", auxLoader.getPosition());
        this.printMotorTelemetry("Intake", auxIntake, DcMotor.RunMode.RUN_USING_ENCODER, INTAKE_COUNTS_PER_REVOLUTION);
        this.printMotorTelemetry("Flywheel", auxFlywheel, DcMotor.RunMode.RUN_USING_ENCODER, FLYWHEEL_COUNTS_PER_REVOLUTION);
        // Update data
        telemetry.update();
    }

    // =============================================================================================
    // END TELEMETRY
    // =============================================================================================

    @Override
    public void loop() {
        gpDriver = super.gamepad1;
        gpDriver_previous = new Gamepad();
        gpOperator = super.gamepad2;
        gpOperator_previous = new Gamepad();

    // =============================================================================================
    // CONTROLS
    // =============================================================================================
        // DRIVER
        CONTROL_DRIVE  = this.deadzone(gpDriver.right_stick_y, DEADZONE_STICK);
        CONTROL_STRAFE = this.deadzone(-gpDriver.left_stick_x, DEADZONE_STICK);
        CONTROL_TWIST  = this.deadzone(-gpDriver.right_stick_x, DEADZONE_STICK);
        CONTROL_FULL_SPEED = gpDriver.right_bumper || gpDriver.dpad_up;
        CONTROL_HALF_SPEED = gpDriver.left_bumper || gpDriver.dpad_down;
        CONTROL_GAS   = gpDriver.a ? 1 : gpDriver.right_trigger;
        CONTROL_BRAKE = gpDriver.y ? 1 : gpDriver.left_trigger;
        CONTROL_DPAD_STRAFE_LEFT = gpDriver.dpad_left;
        CONTROL_DPAD_STRAFE_RIGHT = gpDriver.dpad_right;


        // OPERATOR
        CONTROL_OPERATOR_DRIVE = this.deadzone(gpOperator.right_stick_y, DEADZONE_STICK) * OPERATOR_DRIVE_MODIFIER;
        CONTROL_OPERATOR_TWIST = this.deadzone(-gpOperator.right_stick_x, DEADZONE_STICK) * OPERATOR_DRIVE_MODIFIER;
        CONTROL_FLYWHEEL = gpOperator.right_bumper
                            || gpOperator.a
                            || Math.abs(gpOperator.right_trigger) > DEADZONE_TRIGGER_BUTTON
        ;
        CONTROL_INTAKE =    gpOperator.left_bumper
                            || gpOperator.y
                            || Math.abs(gpOperator.left_trigger) > DEADZONE_TRIGGER_BUTTON
        ;
        CONTROL_INTAKE_VARIABLE = gpOperator.left_stick_y;
        CONTROL_STOPPER = gpOperator.dpad_right || gpOperator.x;
        CONTROL_LOADER = gpOperator.dpad_left || gpOperator.b;

        //===================
        // LEAVE THESE ALONE
        //===================
        if (CONTROL_DRIVE == 0 && CONTROL_STRAFE == 0 && CONTROL_TWIST == 0) {
            CONTROL_DRIVE = CONTROL_OPERATOR_DRIVE;
            CONTROL_TWIST = CONTROL_OPERATOR_TWIST;
        }

        CONTROL_DRIVE = Math.max(-1, Math.min(1, CONTROL_DRIVE));
        CONTROL_STRAFE = Math.max(-1, Math.min(1, CONTROL_STRAFE));
        CONTROL_TWIST = Math.max(-1, Math.min(1, CONTROL_TWIST));
    // =============================================================================================
    // END CONTROLS
    // =============================================================================================

        // Robot Actions
        this.driver();
        this.operator();

        // Telemetry
        this.telecom();

        // Store gamepad states for this session so we can reference them later
        // This is useful for toggle functions with buttons that need context of previous gamepad states
        gpDriver_previous.copy(gpDriver);
        gpOperator_previous.copy(gpOperator);
    }
}