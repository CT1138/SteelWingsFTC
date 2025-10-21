package org.firstinspires.ftc.teamcode.operations.FTC2526.teleop; // package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.core.util.Mecanum;
import org.firstinspires.ftc.teamcode.core.util.PIDdriver;

import java.io.IOException;
@TeleOp(name="Slingshotter", group="Decode")
public class Slingshotter extends OpMode
{
    // define device classes
    PIDdriver moDrivePID;
    Mecanum moMecanum;
    IMU imu;

    // Auxiliary variables
    private boolean isSequenceRunning = false;
    private boolean lastAState = false;

        // {x, y}
        // X = idle, Y = enabled
    private final double[] flywheelPowers = {0.2, 1};
    private final double[] stopperPositions = {0.0, 0.5};
    private final double[] intakePowers = {0.5, 1};

    // Timer for sequencing
    private ElapsedTime moRuntime;
    private ElapsedTime sequenceTimer;
    private int sequenceStep = 0;

    // Actuators
    private DcMotor moDrive_FrontLeft = null;
    private DcMotor moDrive_FrontRight = null;
    private DcMotor moDrive_RearLeft = null;
    private DcMotor moDrive_RearRight = null;
    private DcMotorEx moAux_Flywheel = null;
    private DcMotorEx moAux_Intake = null;
    private Servo soAux_Stopper = null;

    // Editors
    private int pidIndex = 0;
    private final String[] pidNames = {"Proportional", "Integral", "Derivative"};
    private boolean lastRightBumper = false;
    private boolean lastLeftBumper = false;


    // throws IOException as some utility classes I wrote require file operations
    public Slingshotter() throws IOException {
    }

    // Prep our motors and servos
    @Override
    public void init() {
        // Actuator setup
        moDrive_FrontLeft = hardwareMap.get(DcMotor.class, "FL");
        moDrive_FrontRight = hardwareMap.get(DcMotor.class, "FR");
        moDrive_RearLeft = hardwareMap.get(DcMotor.class, "RL");
        moDrive_RearRight = hardwareMap.get(DcMotor.class, "RR");

        moDrive_FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moDrive_FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moDrive_RearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moDrive_RearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        moAux_Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        moAux_Flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        soAux_Stopper = hardwareMap.get(Servo.class, "Stopper");

        moAux_Flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moAux_Flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Start timers
        sequenceTimer = new ElapsedTime();
        moRuntime = new ElapsedTime();

        // PID Drive initialization
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDir = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDir = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logoDir, usbDir)));

        moMecanum = new Mecanum(0.75);
        moDrivePID = new PIDdriver(imu, moMecanum, 0.02, 0, 0.001);


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
        boolean aPressed = gpOperator.a;
        boolean fastIntake = gpOperator.y;

        double flywheelPower = 0;
        double stopperPosition = 0;
        double intakePower = 0;

        intakePower = fastIntake ? intakePowers[0] : intakePowers[1];

        if (!isSequenceRunning) {
            flywheelPower = gpOperator.right_bumper ? flywheelPowers[1] : flywheelPowers[0];
            stopperPosition = gpOperator.left_bumper ? stopperPositions[1] : stopperPositions[0];
        }

        // --- Detect new A press ---
        if (aPressed && !lastAState && !isSequenceRunning) {
            // Start the timed sequence
            isSequenceRunning = true;
            sequenceStep = 0;
            sequenceTimer.reset();
        }
        lastAState = aPressed;

        // --- Sequence logic ---
        if (isSequenceRunning) {
            switch (sequenceStep) {
                case 0:
                    // Step 1: Flywheel to full power
                    flywheelPower = flywheelPowers[1];
                    sequenceTimer.reset();
                    sequenceStep++;
                    break;

                case 1:
                    // Step 2: Wait 0.5 sec
                    if (sequenceTimer.seconds() >= 0.5) {
                        stopperPosition = stopperPositions[0];
                        sequenceTimer.reset();
                        sequenceStep++;
                    }
                    break;

                case 2:
                    // Step 3: Wait 1 sec
                    if (sequenceTimer.seconds() >= 1.0) {
                        stopperPosition = stopperPositions[1];
                        sequenceTimer.reset();
                        sequenceStep++;
                    }
                    break;

                case 3:
                    // Step 4: Wait 1 sec
                    if (sequenceTimer.seconds() >= 1.0) {
                        flywheelPower = flywheelPowers[0];
                        isSequenceRunning = false; // Done!
                    }
                    break;
            }
        }

        // --- Apply outputs ---
        moAux_Flywheel.setPower(flywheelPower);
        moAux_Intake.setPower(intakePower);
        soAux_Stopper.setPosition(stopperPosition);
    }


    private void driver() {
        Gamepad gp = gamepad1;

        double mdDrive = gp.left_stick_y;
        double mdStrafe = -gp.left_stick_x;
        double mdTwist = gp.right_stick_x;
        double mdBrake = gp.right_trigger;

        // Button overrides
        if (gp.dpad_left) mdStrafe = 1 - mdBrake;
        if (gp.dpad_right) mdStrafe = -1 + mdBrake;
        if (gp.dpad_up) mdDrive = -1 + mdBrake;
        if (gp.dpad_down) mdDrive = 1 - mdBrake;
        if (gp.x) mdTwist = -1 + mdBrake;
        if (gp.b) mdTwist = 1 - mdBrake;


        double[] wheelpower = moDrivePID.Calculate(mdDrive, mdStrafe, mdTwist, gamepad2.right_bumper);

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
        telemetry.addData("Flywheel RPM", miFlywheelRPM);
        telemetry.addData("Intake Power", moAux_Intake.getPower());
        // Update data
        telemetry.update();
    }


    public void driverEdit() {
        double kP = moDrivePID.pid.kP;
        double kI = moDrivePID.pid.kI;
        double kD = moDrivePID.pid.kD;

        // --- Selection Switching ---
        if (gamepad1.right_bumper && !lastRightBumper) {
            pidIndex = (pidIndex + 1) % 3; // select next
        } else if (gamepad1.left_bumper && !lastLeftBumper) {
            pidIndex = (pidIndex + 2) % 3; // select last
        }

        // Store current bumper states for edge detection
        lastRightBumper = gamepad1.right_bumper;
        lastLeftBumper = gamepad1.left_bumper;

        // --- Value Adjustment ---
        double adjustSpeed = 0.0005; // increment
        double input = -gamepad1.right_stick_y; // up is positive

        if (Math.abs(input) > 0.05) { // small deadzone
            switch (pidIndex) {
                case 0:
                    kP += input * adjustSpeed;
                    break;
                case 1:
                    kI += input * adjustSpeed;
                    break;
                case 2:
                    kD += input * adjustSpeed;
                    break;
            }
        }

        telemetry.addLine("===================================");
        telemetry.addLine("Editing mode! Use the left and right bumper to select value, right stick Y to adjust.");
        telemetry.addLine(pidNames[pidIndex] + " selected.");
        telemetry.addLine("P Value: " + kP);
        telemetry.addLine("I Value: " + kI);
        telemetry.addLine("D Value: " + kD);
        telemetry.addLine("===================================");

        moDrivePID.pid.adjust(kP, kI, kD);
    }
    // I like to keep loop() fairly minimal and only reference other methods when possible
    @Override
    public void loop() {
        if(gamepad1.start) {
            driverEdit();
        } else {
            driver();
        }

        operator();
        telecom();
    }

}
