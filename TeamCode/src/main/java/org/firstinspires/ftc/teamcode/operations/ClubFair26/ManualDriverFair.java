package org.firstinspires.ftc.teamcode.operations.ClubFair26; // package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.util.Mecanum;

import java.io.IOException;

@Disabled
@TeleOp(name="Club Fair Driver", group="Club Fair 26")
public class ManualDriverFair extends OpMode
{
    // define device classes
    Mecanum moMecanum = new Mecanum(0.5);
    private final ElapsedTime moRuntime = new ElapsedTime();
    private DcMotor moDrive_FrontLeft = null;
    private DcMotor moDrive_FrontRight = null;
    private DcMotor moDrive_RearLeft = null;
    private DcMotor moDrive_RearRight = null;

    // throws IOException as some utility classes I wrote require file operations
    public ManualDriverFair() throws IOException {
    }

    // Prep our motors and servos
    @Override
    public void init() {
        // Single execution on INIT
        moDrive_FrontLeft = hardwareMap.get(DcMotor.class, "FL");
        moDrive_FrontRight = hardwareMap.get(DcMotor.class, "FR");
        moDrive_RearLeft = hardwareMap.get(DcMotor.class, "RL");
        moDrive_RearRight = hardwareMap.get(DcMotor.class, "RR");

        moDrive_FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moDrive_FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moDrive_RearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moDrive_RearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Reset our timer
    @Override
    public void start() {
        moRuntime.reset();
    }

    private void drivetrain() {
        Gamepad driver = gamepad1;
        Gamepad operator = gamepad2;
        // Controls definition
        double mdDrive = driver.left_stick_y;
        double mdStrafe = -driver.left_stick_x;
        double mdTwist = driver.right_stick_x;
        double mdBrake = driver.right_trigger;
        boolean mbDriveLeft = driver.dpad_left;
        boolean mbDriveRight = driver.dpad_right;
        boolean mbDriveUp = driver.dpad_up;
        boolean mbDriveDown = driver.dpad_down;
        boolean mbTwistLeft = driver.x;
        boolean mbTwistRight = driver.b;

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
        drivetrain();
        telecom();
    }

}
