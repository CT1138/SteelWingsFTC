package org.firstinspires.ftc.teamcode.FTC2425.TeleOP; // package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.util.Mecanum;

import java.io.IOException;

@TeleOp(name="Manual Driver", group="Into-The-Deep")
@Disabled
public class ManualDriver extends OpMode
{
    // define the motors and whatnot
    Mecanum moMecanum = new Mecanum(0.75);
    private final ElapsedTime moRuntime = new ElapsedTime();
    private DcMotor moDrive_FrontLeft = null;
    private DcMotor moDrive_FrontRight = null;
    private DcMotor moDrive_RearLeft = null;
    private DcMotor moDrive_RearRight = null;
    private DcMotor moArm_Extend = null;
    private DcMotor moArm_PhaseTwo = null;

    private DcMotor moArm_Twist = null;

    private Servo moServoClaw = null;
    private TouchSensor moLiftarmZero = null;

    public ManualDriver() throws IOException {
    }

    @Override
    public void init() {
        // Single execution on INIT
        moDrive_FrontLeft = hardwareMap.get(DcMotor.class, "Drive_FrontLeft");
        moDrive_FrontRight = hardwareMap.get(DcMotor.class, "Drive_FrontRight");
        moDrive_RearLeft = hardwareMap.get(DcMotor.class, "Drive_RearLeft");
        moDrive_RearRight = hardwareMap.get(DcMotor.class, "Drive_RearRight");

        moArm_Extend = hardwareMap.get(DcMotor.class, "Arm_Extend");
        moArm_PhaseTwo = hardwareMap.get(DcMotor.class, "Arm_PhaseTwo");
        moArm_Twist = hardwareMap.get(DcMotor.class, "Arm_Twist");

        moServoClaw = hardwareMap.get(Servo.class, "Servo_Claw");
        moLiftarmZero = hardwareMap.get(TouchSensor.class, "TouchSensor");

        moDrive_FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moDrive_FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moDrive_RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moDrive_RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        moArm_Extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moArm_PhaseTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        moArm_Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        moDrive_FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moDrive_FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moDrive_RearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moDrive_RearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void start() {
        moRuntime.reset();
        moServoClaw.setPosition(1);
    }

    private void liftarm() {
        // Arm Lift power
        double mdElbowPower = 0;
        mdElbowPower = -gamepad2.right_stick_y;
        if(!gamepad2.left_bumper) mdElbowPower = mdElbowPower * 0.5;
        moArm_PhaseTwo.setPower(mdElbowPower);

        // Arm extension
        double mdExtendPower = gamepad2.left_stick_y;
        if (moLiftarmZero.isPressed() && gamepad2.left_stick_y > 0) {
            mdExtendPower = 0;
        }
        moArm_Extend.setPower(mdExtendPower);

        // Arm Twist using power and position limits

        moArm_Twist.setPower(gamepad2.left_stick_x / 4);

        if (gamepad2.dpad_up || gamepad2.dpad_down) {
            if (gamepad2.dpad_up) {
                moServoClaw.setPosition(-1);
            }

            if (gamepad2.dpad_down) {
                moServoClaw.setPosition(1);
            }
        }
        telemetry.addLine("===================================");
        telemetry.addLine("Arm Positions");
        telemetry.addLine("===================================");
        telemetry.addData("Arm Extend Position", moArm_Extend.getCurrentPosition());
        telemetry.addData("Arm Elbow Position", moArm_PhaseTwo.getCurrentPosition());
        telemetry.addData("Arm Twist Position", moArm_Twist.getCurrentPosition());
        telemetry.addData("Arm Claw Position", moServoClaw.getPosition());
        telemetry.addLine("===================================");
    }

    private void drivetrain() {
        double mdDrive = gamepad1.left_stick_y;
        double mdStrafe = -gamepad1.left_stick_x;
        double mdTwist = gamepad1.right_stick_x;

        boolean mbSlow = gamepad1.right_bumper;
        boolean mbSlower = gamepad1.left_bumper;

        double mdBrake = gamepad1.right_trigger;

        if (gamepad1.dpad_left) mdStrafe = 1 - mdBrake;
        if (gamepad1.dpad_right) mdStrafe = -1 + mdBrake;
        if (gamepad1.dpad_up) mdDrive = -1 + mdBrake;
        if (gamepad1.dpad_down) mdDrive = 1 - mdBrake;

        if (gamepad1.x) mdTwist = -1 + mdBrake;
        if (gamepad1.b) mdTwist = 1 - mdBrake;

        if (mbSlower) {
            mdDrive = mdDrive * 0.5;
            mdStrafe = mdStrafe * 0.5;
            mdTwist = mdStrafe * 0.5;
        }

        if (!mbSlow) {
            mdDrive = mdDrive * 0.75;
            mdStrafe = mdStrafe * 0.75;
            mdTwist = mdTwist * 0.75;
        }

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
        liftarm();
        drivetrain();
        telemetry.update();
    }

}
