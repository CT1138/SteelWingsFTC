package org.firstinspires.ftc.teamcode.ClubFair26; // package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.util.Mecanum;

import java.io.IOException;

@TeleOp(name="Club Fair Driver", group="Club Fair 26")
public class ManualDriverFair extends OpMode
{
    // define the motors and whatnot
    Mecanum moMecanum = new Mecanum(0.75);
    private final ElapsedTime moRuntime = new ElapsedTime();
    private DcMotor moDrive_FrontLeft = null;
    private DcMotor moDrive_FrontRight = null;
    private DcMotor moDrive_RearLeft = null;
    private DcMotor moDrive_RearRight = null;

    public ManualDriverFair() throws IOException {
    }

    @Override
    public void init() {
        // Single execution on INIT
        moDrive_FrontLeft = hardwareMap.get(DcMotor.class, "FL");
        moDrive_FrontRight = hardwareMap.get(DcMotor.class, "FR");
        moDrive_RearLeft = hardwareMap.get(DcMotor.class, "RL");
        moDrive_RearRight = hardwareMap.get(DcMotor.class, "RR");

        moDrive_FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moDrive_FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moDrive_RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moDrive_RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        moDrive_FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moDrive_FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moDrive_RearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moDrive_RearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void start() {
        moRuntime.reset();
    }

    private void drivetrain() {
        double mdDrive = gamepad1.left_stick_y;
        double mdStrafe = -gamepad1.left_stick_x;
        double mdTwist = gamepad1.right_stick_x;

        double mdBrake = gamepad1.right_trigger;

        if (gamepad1.dpad_left) mdStrafe = 1 - mdBrake;
        if (gamepad1.dpad_right) mdStrafe = -1 + mdBrake;
        if (gamepad1.dpad_up) mdDrive = -1 + mdBrake;
        if (gamepad1.dpad_down) mdDrive = 1 - mdBrake;

        if (gamepad1.x) mdTwist = -1 + mdBrake;
        if (gamepad1.b) mdTwist = 1 - mdBrake;

        double[] wheelpower = moMecanum.Calculate(mdDrive, mdStrafe, -mdTwist, gamepad2.right_bumper);

        moDrive_FrontLeft.setPower(wheelpower[0]);
        moDrive_FrontRight.setPower(wheelpower[1]);
        moDrive_RearLeft.setPower(wheelpower[2]);
        moDrive_RearRight.setPower(wheelpower[3]);

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
    }

    @Override
    public void loop() {
        drivetrain();
        telemetry.update();
    }

}
