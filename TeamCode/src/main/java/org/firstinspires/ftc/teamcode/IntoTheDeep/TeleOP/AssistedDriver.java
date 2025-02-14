package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOP; // package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.clubFair.FairObjectives;

import org.firstinspires.ftc.teamcode.core.Objective;
import org.firstinspires.ftc.teamcode.util.Mecanum;

import java.util.Objects;

@TeleOp(name="Assisted Driver", group="Into-The-Deep")
public class AssistedDriver extends OpMode
{
    // define the motors and whatnot
    Objective currentObjective = null;
    Mecanum mecanum = new Mecanum();
    private DcMotor[] miMotors;
    private Servo[] miServos;
    private FairObjectives fairObjectives;
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime objectiveRuntime = new ElapsedTime();
    private DcMotor Drive_FrontLeft = null;
    private DcMotor Drive_FrontRight = null;
    private DcMotor Drive_RearLeft = null;
    private DcMotor Drive_RearRight = null;
    private DcMotor Arm_Extend = null;
    private DcMotor Arm_PhaseTwo = null;
    private DcMotor Arm_Twist = null;
    private Servo ServoClaw = null;
    private TouchSensor LiftarmStop = null;

    @Override
    public void init() {
        // Single execution on INIT
        Drive_FrontLeft  = hardwareMap.get(DcMotor.class, "Drive_FrontLeft");
        Drive_FrontRight = hardwareMap.get(DcMotor.class, "Drive_FrontRight");
        Drive_RearLeft   = hardwareMap.get(DcMotor.class, "Drive_RearLeft");
        Drive_RearRight  = hardwareMap.get(DcMotor.class, "Drive_RearRight");

        Arm_Extend = hardwareMap.get(DcMotor.class, "Arm_Extend");
        Arm_PhaseTwo = hardwareMap.get(DcMotor.class, "Arm_PhaseTwo");
        Arm_Twist = hardwareMap.get(DcMotor.class, "Arm_Twist");

        ServoClaw = hardwareMap.get(Servo.class, "Servo_Claw");
        LiftarmStop = hardwareMap.get(TouchSensor.class, "TouchSensor");

        Drive_FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Drive_FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Drive_RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Drive_RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Arm_Extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm_PhaseTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Arm_Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm_Extend.setDirection(DcMotorSimple.Direction.REVERSE);

        Drive_FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Drive_FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Drive_RearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Drive_RearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        miMotors = new DcMotor[]{Arm_Extend, Arm_PhaseTwo, Arm_Twist};
        miServos = new Servo[]{ServoClaw};

        fairObjectives = new FairObjectives(miMotors, miServos);
    }

    @Override
    public void start() {
        objectiveRuntime.reset();
        runtime.reset();
        ServoClaw.setPosition(1);
    }

    private boolean canMoveOn() {
        for (DcMotor mdMotor : miMotors) if (mdMotor.isBusy() && mdMotor.getCurrentPosition() != mdMotor.getTargetPosition()) return false;
        return true;
    }

    private void liftarmMaster() {
            boolean mbFloor = LiftarmStop.isPressed();
            int miButtonPressDelay = 1;

        // Objective Declarations
            Objective foZero = fairObjectives.foZero(1);
            Objective foWave = fairObjectives.foWave(0.75);
            Objective foGrabFromFloor = fairObjectives.foGrabFromFloor(1);
            Objective foGrabFromLowerRung = fairObjectives.foGrabFromLowerRung(1);

        // ========================
        // BEGIN OBJECTIVE CONTROLS
        // ========================

        // STOP EVERYTHING
        // Control = Left Bumper + Right Bumper + Back
            if (gamepad2.left_bumper && gamepad2.right_bumper && gamepad2.back) {
                if (currentObjective != null) currentObjective.stop();

                Arm_Extend.setPower(0);
                Arm_Twist.setPower(0);
                Arm_PhaseTwo.setPower(0);
                Arm_Extend.setTargetPosition(Arm_Extend.getCurrentPosition());
                Arm_Twist.setTargetPosition(Arm_Twist.getCurrentPosition());
                Arm_PhaseTwo.setTargetPosition(Arm_PhaseTwo.getCurrentPosition());
            }

        // Wave at the viewer!
        // Control = A
            if (canMoveOn() && gamepad2.a && runtime.seconds() > miButtonPressDelay) {
                runtime.reset();
                currentObjective = foWave;
                foWave.run(objectiveRuntime, mbFloor);
            }

        // Grab Sample From Lower Rung
        // Control = B + Dpad_Left
            if (canMoveOn() && gamepad2.b && gamepad2.dpad_left && runtime.seconds() > miButtonPressDelay) {
                runtime.reset();
                currentObjective = foGrabFromLowerRung;
                foGrabFromLowerRung.run(objectiveRuntime, mbFloor);
            }

        // Grab Sample From Floor
        // Control = B + Dpad_Down
            if (canMoveOn() && gamepad2.b && gamepad2.dpad_down && runtime.seconds() > miButtonPressDelay) {
                runtime.reset();
                currentObjective = foGrabFromFloor;
                foGrabFromFloor.run(objectiveRuntime, mbFloor);
            }

        // Return Motors to Zero
        // Control = Back
            if (canMoveOn() && gamepad2.back && runtime.seconds() > miButtonPressDelay) {
                runtime.reset();
                currentObjective = foZero;
                foZero.run(objectiveRuntime, mbFloor);
            }

        // ======================
        // END OBJECTIVE CONTROLS
        // ======================
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

        double mdSlowerPowerModifier = 0.5;
        if (slower) {
            drive = drive * mdSlowerPowerModifier;
            strafe = strafe * mdSlowerPowerModifier;
            twist = strafe * mdSlowerPowerModifier;
        }

        double mdSlowPowerModifier = 0.75;
        if (!slow) {
            drive = drive * mdSlowPowerModifier;
            strafe = strafe * mdSlowPowerModifier;
            twist = twist * mdSlowPowerModifier;
        }

        double[] wheelpower = mecanum.calculate(drive, strafe, -twist, gamepad2.right_bumper);

        Drive_FrontLeft.setPower(wheelpower[0]);
        Drive_FrontRight.setPower(wheelpower[1]);
        Drive_RearLeft.setPower(wheelpower[2]);
        Drive_RearRight.setPower(wheelpower[3]);
    }

    @Override
    public void loop() {
        liftarmMaster();
        drivetrainMaster();

        telemetry.addLine("===================================");
        telemetry.addLine("Arm Positions");
        telemetry.addLine("===================================");
        telemetry.addData("Arm Extend Position", Arm_Extend.getCurrentPosition());
        telemetry.addData("Arm Extend Target Position", Arm_Extend.getTargetPosition());
        telemetry.addData("Arm Elbow Position", Arm_PhaseTwo.getCurrentPosition());
        telemetry.addData("Arm Elbow Target Position", Arm_PhaseTwo.getTargetPosition());
        telemetry.addData("Arm Twist Position", Arm_Twist.getCurrentPosition());
        telemetry.addData("Arm Twist Target Position", Arm_Twist.getTargetPosition());
        telemetry.addData("Arm Claw Position", ServoClaw.getPosition());
        telemetry.addLine("===================================");
        telemetry.addLine("");
        telemetry.addLine("===================================");
        telemetry.addLine("DriveTrain");
        telemetry.addLine("===================================");
        telemetry.addData("Front Left Power", Drive_FrontLeft.getPower());
        telemetry.addData("Front Right Power", Drive_FrontRight.getPower());
        telemetry.addData("Rear Left Power", Drive_RearLeft.getPower());
        telemetry.addData("Rear Right Power", Drive_RearRight.getPower());
        telemetry.addLine("");
        telemetry.addData("Front Left Position", Drive_FrontLeft.getCurrentPosition());
        telemetry.addData("Front Right Position", Drive_FrontRight.getCurrentPosition());
        telemetry.addData("Rear Left Position", Drive_RearLeft.getCurrentPosition());
        telemetry.addData("Rear Right Position", Drive_RearRight.getCurrentPosition());
        telemetry.addLine("===================================");

        telemetry.update();
    }

}
