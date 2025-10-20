package org.firstinspires.ftc.teamcode.operations.FTC2526.teleop; // package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.util.Mecanum;

import java.io.IOException;
import java.nio.channels.Selector;

@TeleOp(name="Slingshotter", group="Decode")
public class Slingshotter extends OpMode
{
    // define device classes
    Mecanum moMecanum = new Mecanum(0.75);

    private boolean flywheelOn = false; // Current toggle state
    private boolean lastAState = false; // What the button was doing last loop

    private final ElapsedTime moRuntime = new ElapsedTime();
    private DcMotor moDrive_FrontLeft = null;
    private DcMotor moDrive_FrontRight = null;
    private DcMotor moDrive_RearLeft = null;
    private DcMotor moDrive_RearRight = null;
    private DcMotor moAux_Flywheel = null;
    private Servo soAux_Stopper = null;

    // throws IOException as some utility classes I wrote require file operations
    public Slingshotter() throws IOException {
    }

    // Prep our motors and servos
    @Override
    public void init() {
        // Drivetrain Setup
        moDrive_FrontLeft = hardwareMap.get(DcMotor.class, "FL");
        moDrive_FrontRight = hardwareMap.get(DcMotor.class, "FR");
        moDrive_RearLeft = hardwareMap.get(DcMotor.class, "RL");
        moDrive_RearRight = hardwareMap.get(DcMotor.class, "RR");

        moDrive_FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moDrive_FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moDrive_RearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moDrive_RearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        moAux_Flywheel = hardwareMap.get(DcMotor.class, "Flywheel");
        soAux_Stopper = hardwareMap.get(Servo.class, "Stopper");

        moAux_Flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moAux_Flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    // Reset our timer
    @Override
    public void start() {
        moRuntime.reset();
    }

    private void operator() {
        Gamepad gpOperator = gamepad2;

        boolean aPressed = gpOperator.a;

        if (aPressed && !lastAState) {
            flywheelOn = !flywheelOn;
        }


        lastAState = aPressed;
        
        double flywheelPower = flywheelOn ? 1.0 : 0.1;
        moAux_Flywheel.setPower(flywheelPower);

        boolean mbStopper = gpOperator.dpad_up;
        soAux_Stopper.setPosition(mbStopper ? 0.5 : 0.0);
    }

    private void driver() {
        Gamepad gpDriver = gamepad1;
        // Controls definition
        double mdDrive = gpDriver.left_stick_y;
        double mdStrafe = -gpDriver.left_stick_x;
        double mdTwist = gpDriver.right_stick_x;
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

        if (mbTwistLeft) mdTwist = -1 + mdBrake;
        if (mbTwistRight) mdTwist = 1 - mdBrake;

        // Calculate the target power for all wheels assuming a mecanum drivetrain is in use
        // see teamcode.core.util.Mecanum
        double[] wheelpower = moMecanum.Calculate(mdTwist, mdStrafe, -mdDrive, gamepad2.right_bumper);

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
        telemetry.addData("Front Left Power", moDrive_FrontLeft.getPower());
        telemetry.addData("Front Right Power", moDrive_FrontRight.getPower());
        telemetry.addData("Rear Left Power", moDrive_RearLeft.getPower());
        telemetry.addData("Rear Right Power", moDrive_RearRight.getPower());

        telemetry.addData("Front Left Position", moDrive_FrontLeft.getCurrentPosition());
        telemetry.addData("Front Right Position", moDrive_FrontRight.getCurrentPosition());
        telemetry.addData("Rear Left Position", moDrive_RearLeft.getCurrentPosition());
        telemetry.addData("Rear Right Position", moDrive_RearRight.getCurrentPosition());
        telemetry.addLine("===================================");

        telemetry.update();
    }

    // I like to keep loop() fairly minimal and only reference other methods
    @Override
    public void loop() {
        driver();
        operator();
        telecom();
    }

}
