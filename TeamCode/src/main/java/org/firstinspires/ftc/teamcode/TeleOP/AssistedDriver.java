package org.firstinspires.ftc.teamcode.TeleOP; // package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.ConfigManager;

import java.io.IOException;

@TeleOp(name="Main TeleOP", group="Into-The-Deep")
public class AssistedDriver extends OpMode
{
    // define the motors and whatnot
    ConfigManager config = new ConfigManager("TeamCode/src/main/res/values/robot.properties");
    ConfigManager devices = new ConfigManager("TeamCode/src/main/res/values/devices.properties");
    private final int ARM_TWIST_MIN   = config.getInt("ARM_TWIST_MIN"); //-140; // Equivalent to -180 degrees
    private final int ARM_TWIST_MAX   = config.getInt("ARM_TWIST_MAX"); //140;  // Equivalent to 180 degrees
    private int armTwistStartingPosition     = config.getInt("ARM_TWIST_START");

    private final int ARM_EXTEND_MIN = config.getInt("ARM_EXTEND_MIN");
    private final int ARM_EXTEND_MAX = config.getInt("ARM_EXTEND_MAX");
    private int armExtendStartingPosition   = config.getInt("ARM_EXTEND_START");


    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor Drive_FrontLeft = null;
    private DcMotor Drive_FrontRight = null;
    private DcMotor Drive_RearLeft = null;
    private DcMotor Drive_RearRight = null;
    private DcMotor Arm_Extend = null;
    private DcMotor Arm_PhaseTwo = null;
    private DcMotor Arm_Twist = null;

    private Servo ServoClaw = null;
    private TouchSensor LiftarmStop = null;

    public AssistedDriver() throws IOException {
    }

    @Override
    public void init() {
        // Single execution on INIT
        Drive_FrontLeft  = hardwareMap.get(DcMotor.class, devices.getString("FrontLeft"));
        Drive_FrontRight = hardwareMap.get(DcMotor.class, devices.getString("FrontRight"));
        Drive_RearLeft   = hardwareMap.get(DcMotor.class, devices.getString("RearLeft"));
        Drive_RearRight  = hardwareMap.get(DcMotor.class, devices.getString("RearRight"));

        Arm_Extend = hardwareMap.get(DcMotor.class, devices.getString("ArmExtend"));
        Arm_PhaseTwo = hardwareMap.get(DcMotor.class, devices.getString("ArmElbow"));
        Arm_Twist = hardwareMap.get(DcMotor.class, devices.getString("ArmTwist"));

        ServoClaw = hardwareMap.get(Servo.class, devices.getString("Claw"));
        LiftarmStop = hardwareMap.get(TouchSensor.class, devices.getString("Stopper"));

        Drive_FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Drive_FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Drive_RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Drive_RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Arm_Extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm_PhaseTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm_Twist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Arm_Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm_Twist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Drive_FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Drive_FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Drive_RearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Drive_RearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void start() {
        runtime.reset();
        ServoClaw.setPosition(1);
        armTwistStartingPosition = Arm_Twist.getCurrentPosition();
    }

    private double[] calculateMecanum(double drive, double strafe, double twist) {
        double frontleft = drive + strafe + twist;
        double frontright = drive - strafe - twist;
        double rearleft = drive - strafe + twist;
        double rearright = drive + strafe - twist;
        return new double[]{-frontleft, frontright, -rearleft, rearright};
    }

    private void liftarmMaster() {
        // Arm Lift power
        double elbowPower = 0;
        elbowPower = -gamepad2.right_stick_y;
        if(!gamepad2.left_bumper) elbowPower = elbowPower * 0.5;
        Arm_PhaseTwo.setPower(elbowPower);

        // Arm extension
        double extendPower = gamepad2.left_stick_y;
        if (LiftarmStop.isPressed() && gamepad2.left_stick_y > 0) {
            extendPower = 0;
        }
        Arm_Extend.setPower(extendPower);

        // Arm Twist using power and position limits
        double twistPower = 0.0;
        if (gamepad2.dpad_left) {
            twistPower = 0.5;
        } else if (gamepad2.dpad_right) {
            twistPower = -0.5;
        }

        int currentPosition = Arm_Twist.getCurrentPosition();
        if ((currentPosition <= armTwistStartingPosition + ARM_TWIST_MAX && twistPower > 0) ||
            (currentPosition >= armTwistStartingPosition + ARM_TWIST_MIN && twistPower < 0)) {
            Arm_Twist.setPower(twistPower);
        } else {
            Arm_Twist.setPower(0.0);
        }

        if (gamepad2.dpad_up || gamepad2.dpad_down) {
            if (gamepad2.dpad_up) {
                ServoClaw.setPosition(-1);
            }

            if (gamepad2.dpad_down) {
                ServoClaw.setPosition(1);
            }
        }

        telemetry.addData("Arm Extend Position", Arm_Extend.getCurrentPosition());
        telemetry.addData("Arm Elbow Power", Arm_PhaseTwo.getPower());
        telemetry.addData("Arm Twist Position", Arm_Twist.getCurrentPosition());
        telemetry.addData("Arm Twist Starting Position", armTwistStartingPosition);
        telemetry.addData("Claw Position", ServoClaw.getPosition());
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

        double[] wheelpower = calculateMecanum(drive, strafe, -twist);

        Drive_FrontLeft.setPower(wheelpower[0]);
        Drive_FrontRight.setPower(wheelpower[1]);
        Drive_RearLeft.setPower(wheelpower[2]);
        Drive_RearRight.setPower(wheelpower[3]);

        telemetry.addData("Front Left Power", Drive_FrontLeft.getPower());
        telemetry.addData("Front Right Power", Drive_FrontRight.getPower());
        telemetry.addData("Rear Left Power", Drive_RearLeft.getPower());
        telemetry.addData("Rear Right Power", Drive_RearRight.getPower());

        telemetry.addData("Front Left Position", Drive_FrontLeft.getCurrentPosition());
        telemetry.addData("Front Right Position", Drive_FrontRight.getCurrentPosition());
        telemetry.addData("Rear Left Position", Drive_RearLeft.getCurrentPosition());
        telemetry.addData("Rear Right Position", Drive_RearRight.getCurrentPosition());
    }

    @Override
    public void loop() {
        liftarmMaster();
        drivetrainMaster();
        telemetry.update();
    }

    @Override
    public void stop() {}
}
