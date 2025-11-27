package org.firstinspires.ftc.teamcode.operations.FTC2526.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.core.util.Mecanum;

import java.io.IOException;
import java.lang.reflect.Field;
import java.util.Arrays;

@Disabled
@TeleOp(name="Ignore Me", group="Decode")
public class TeleOPMaster extends OpMode
{
    //============================================================================================
    // BEGIN VARIABLES
    // EDIT CONTROLS IN loop()
    //============================================================================================
    // SETTINGS
    // CONSTANTS
    public final double    DEADZONE_TRIGGER_BUTTON = 0.5;
    public final double    DEADZONE_STICK = 0.05;
    public final double    OPERATOR_DRIVE_MODIFIER = 0.5;
    public final double    DRIVE_BASE_SPEED = 0.8;
    public final double    STRAFE_BUTTON_SPEED = 0.5;
    public final double    TWIST_BASE_SPEED = 0.5;
    public final double    STRAFE_BASE_SPEED = 0.5;

    public final int       DRIVE_COUNTS_PER_REVOLUTION = 28;
    public final int       FLYWHEEL_COUNTS_PER_REVOLUTION = 28;
    public final int       INTAKE_COUNTS_PER_REVOLUTION = 28;

    public final int       DRIVE_MAX_RPM = 5500;
    public final double    DRIVE_SLIP_THRESHOLD = 1.2;

    public final int       FLYWHEEL_MIN_RPM = 0;
    public final int       FLYWHEEL_MAX_RPM = 5500;

    public final double PUSHER_OPEN_POSITION = 0.9;
    public final double PUSHER_CLOSED_POSITION = 0.52;

    public final double    LOADER_OPEN_POSITION = 0.3;
    public final double    LOADER_CLOSED_POSITION = 0.45;

    // =============================================================================================
    public double    flywheelVelocity;
    public double    loaderPosition = LOADER_CLOSED_POSITION;
    public double    pusherPosition = PUSHER_CLOSED_POSITION;

    public ControlState cDriver;
    public ControlState cOperator;

    public final LogoFacingDirection CONTROL_LOGO_DIRECTION = LogoFacingDirection.LEFT;
    public final UsbFacingDirection CONTROL_USB_DIRECTION = UsbFacingDirection.UP;
    public final LogoFacingDirection EXPANSION_LOGO_DIRECTION = LogoFacingDirection.FORWARD;
    public final UsbFacingDirection EXPANSION_USB_DIRECTION = UsbFacingDirection.DOWN;
    // END CONSTANTS

    // PIDF Settings:
    // (P)roportional, (I)ntegral, (D)erivative, and (F)loat
    // P*: how strongly the controller reacts to the current error
    // I*: how strongly the controller reacts to the accumulation of past error (helps eliminate steady-state error)
    // D*: how strongly the controller reacts to the rate of change of the error (helps reduce overshoot and oscillation)
    // F:  provides a baseline output to reach a target speed or position without error
    public final PIDFCoefficients DRIVE_PIDF = new PIDFCoefficients(15.0, 3.0, 0.0, 0.0);
    public final PIDFCoefficients FLYWHEEL_PIDF = new PIDFCoefficients(20.0, 5.0, 1.0, 0.0);
    public final PIDFCoefficients INTAKE_PIDF = new PIDFCoefficients(10.0, 3.0, 0.0, 0.0);
    // END PIDF SETTINGS

    // SYSTEM VARIABLES -- LEAVE BE
    public Gamepad gpDriver;
    public Gamepad gpDriver_previous;
    public Gamepad gpOperator;
    public Gamepad gpOperator_previous;

    public Mecanum         mecanum;
    public IMU             Control_IMU;
    public IMU             Expansion_IMU;
    public ElapsedTime     elapsedTime;
    public ElapsedTime     rampTime;
    public DcMotorEx       driveFL = null;
    public DcMotorEx       driveFR = null;
    public DcMotorEx       driveRL = null;
    public DcMotorEx       driveRR = null;
    public DcMotorEx       auxFlywheel = null;
    public DcMotorEx       auxIntake = null;
    public Servo           auxLoader = null;
    public Servo           auxPusher = null;

    //============================================================================================
    // END VARIABLES
    //============================================================================================

    // Throws IOException as some utility classes I wrote require file operations
    public TeleOPMaster() throws IOException {
    }

    // =============================================================================================
    // HELPER METHODS
    // =============================================================================================
    public double rpmToVelocity(double RPM, int countsPerRevolution) {
        return ( RPM / 60 ) * countsPerRevolution;
    }

    public double velocityToRPM(double Velocity, int countsPerRevolution) {
        return ( Velocity / countsPerRevolution ) * 60;
    }
    public double deadzone(double input, double threshold) {
        return Math.abs(input) < threshold ? 0 : input;
    }
    public boolean pressedThisFrame(boolean current, boolean previous) {
        return current && !previous;
    }
    public boolean releasedThisFrame(boolean current, boolean previous) {
        return !current && previous;
    }

    // =============================================================================================
    // END HELPER METHODS
    // =============================================================================================

    // Prep our motors and servos
    @Override
    public void init() {
        gpDriver = gamepad1;
        gpDriver_previous = new Gamepad();
        gpOperator = gamepad2;
        gpOperator_previous = new Gamepad();

        // Actuator setup
        this.driveFL = hardwareMap.get(DcMotorEx.class, "FL");
        this.driveFR = hardwareMap.get(DcMotorEx.class, "FR");
        this.driveRL = hardwareMap.get(DcMotorEx.class, "RL");
        this.driveRR = hardwareMap.get(DcMotorEx.class, "RR");

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

        this.auxIntake = hardwareMap.get(DcMotorEx.class, "Intake");
        auxIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        auxIntake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, INTAKE_PIDF);
        auxIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        auxIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        this.auxFlywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        auxFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        auxFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, FLYWHEEL_PIDF);
        auxFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        auxFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);

        this.auxLoader = hardwareMap.get(Servo.class, "Loader");
        this.auxPusher = hardwareMap.get(Servo.class, "Pusher");

        // IMU Initialization
        this.Control_IMU = hardwareMap.get(IMU.class, "imu");
        Control_IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(CONTROL_LOGO_DIRECTION, CONTROL_USB_DIRECTION)));

        // Drivetrain handlers
        this.mecanum = new Mecanum();

        // Start timers
        this.elapsedTime = new ElapsedTime();
        this.rampTime = new ElapsedTime();
    }

    // Reset our timers
    @Override
    public void start() {
        elapsedTime.reset();
        rampTime.reset();
    }

    public void feedback(Gamepad[] pads, Gamepad[] prevPads, int rumbleTime, double rumbleIntensity) {
        try {
            Field[] fields = Gamepad.class.getFields();

            for (int i = 0; i < pads.length; i++) {
                Gamepad current = pads[i];
                Gamepad previous = prevPads[i];

                for (Field field : fields) {
                    if (field.getType() != boolean.class) continue;

                    boolean currVal = field.getBoolean(current);
                    boolean prevVal = field.getBoolean(previous);

                    if (this.pressedThisFrame(currVal, prevVal)) {
                        // Rumble this specific gamepad
                        current.rumble(rumbleIntensity, rumbleIntensity, rumbleTime);
                        break; // only 1 rumble per frame per gamepad
                    }
                }
            }

        } catch (IllegalAccessException e) {
            telemetry.addData("Rumble Error", e.getMessage());
        }
    }


    // =============================================================================================
    // OPERATOR METHODS
    // =============================================================================================
    public void operator() {
    }
    // =============================================================================================
    // END OPERATOR METHODS
    // =============================================================================================

    // =============================================================================================
    // DRIVER METHODS
    // =============================================================================================
    public double getHeading() {
        return Control_IMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public double[] tractionControl(double[] powers, double slipThreshold) {
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

        telemetry.addData("Traction Control Factors", Arrays.toString(powers));
        return powers;
    }

    public double driveModifier(boolean full, boolean half,double gas, double brake, double base) {
        double modifier = base + (gas * (1 - base)) - (brake * base);

        if (full) return 1;
        if (half) return 0.5;
        return Math.max(0, Math.min(1, modifier));
    }

    public void driver() {
    }
    // =============================================================================================
    // END DRIVER METHODS
    // =============================================================================================


    // =============================================================================================
    // TELEMETRY
    // =============================================================================================
    public void printMotorTelemetry(String name, DcMotorEx motor, DcMotor.RunMode runMode, int counts) {
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
    public void telecom() {
        telemetry.addData("Loader Position", auxLoader.getPosition());
        telemetry.addData("Pusher Position", auxPusher.getPosition());
        this.printMotorTelemetry("Intake", auxIntake, DcMotor.RunMode.RUN_USING_ENCODER, INTAKE_COUNTS_PER_REVOLUTION);
        this.printMotorTelemetry("Flywheel", auxFlywheel, DcMotor.RunMode.RUN_USING_ENCODER, FLYWHEEL_COUNTS_PER_REVOLUTION);
        this.printMotorTelemetry("Front Left", driveFL, DcMotor.RunMode.RUN_USING_ENCODER, DRIVE_COUNTS_PER_REVOLUTION);
        this.printMotorTelemetry("Front Right", driveFR, DcMotor.RunMode.RUN_USING_ENCODER, DRIVE_COUNTS_PER_REVOLUTION);
        this.printMotorTelemetry("Rear Left", driveRL, DcMotor.RunMode.RUN_USING_ENCODER, DRIVE_COUNTS_PER_REVOLUTION);
        this.printMotorTelemetry("Rear Right", driveRR, DcMotor.RunMode.RUN_USING_ENCODER, DRIVE_COUNTS_PER_REVOLUTION);
        // Update data
        telemetry.update();
    }

    // =============================================================================================
    // END TELEMETRY
    // =============================================================================================

    public void controls() {
    }

    @Override
    public void loop() {
        this.controls();
        this.feedback(
                new Gamepad[]{gpDriver, gpOperator},
                new Gamepad[]{gpDriver_previous, gpOperator_previous},
                50,
                1);
        // Robot Actions
        this.driver();
        this.operator();

        // Telemetry
        this.telecom();
        gpDriver_previous.copy(gpDriver);
        gpOperator_previous.copy(gpOperator);
    }
}