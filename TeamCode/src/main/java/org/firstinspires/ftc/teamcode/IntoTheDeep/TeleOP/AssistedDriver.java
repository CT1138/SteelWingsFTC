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

@TeleOp(name="Assisted Driver", group="Into-The-Deep")
public class AssistedDriver extends OpMode
{
    // define the motors and whatnot
    Objective moCurrentObjective = null;
    Mecanum moMecanum = new Mecanum(0.75);
    private DcMotor[] moMotors;
    private Servo[] moServos;
    private FairObjectives moFairObjectives;
    private final ElapsedTime moRuntime = new ElapsedTime();
    private final ElapsedTime moObjectiveRuntime = new ElapsedTime();
    private DcMotor moDrive_FrontLeft = null;
    private DcMotor moDrive_FrontRight = null;
    private DcMotor moDrive_RearLeft = null;
    private DcMotor moDrive_RearRight = null;
    private DcMotor moArm_Extend = null;
    private DcMotor moArm_PhaseTwo = null;
    private DcMotor moArm_Twist = null;
    private Servo moServoClaw = null;
    private TouchSensor moLiftarmZero = null;

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
        moArm_Extend.setDirection(DcMotorSimple.Direction.REVERSE);

        moDrive_FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moDrive_FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moDrive_RearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moDrive_RearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moMotors = new DcMotor[]{moArm_Extend, moArm_PhaseTwo, moArm_Twist};
        moServos = new Servo[]{moServoClaw};

        moFairObjectives = new FairObjectives(moMotors, moServos);
    }

    @Override
    public void start() {
        moObjectiveRuntime.reset();
        moRuntime.reset();
        moServoClaw.setPosition(1);
    }

    private boolean canMoveOn() {
        for (DcMotor moMotor : moMotors) if (moMotor.isBusy() && moMotor.getCurrentPosition() != moMotor.getTargetPosition()) return false;
        return true;
    }

    private void liftarmMaster() {
            boolean mbFloor = moLiftarmZero.isPressed();
            int miButtonPressDelay = 1;

        // Objective Declarations
            Objective foZero = moFairObjectives.foZero(1);
            Objective foWave = moFairObjectives.foWave(0.75);
            Objective foGrabFromFloor = moFairObjectives.foGrabFromFloor(1);
            Objective foGrabFromLowerRung = moFairObjectives.foGrabFromLowerRung(1);

        // ========================
        // BEGIN OBJECTIVE CONTROLS
        // ========================

        // STOP EVERYTHING
        // Control = Left Bumper + Right Bumper + Back
            if (gamepad2.left_bumper && gamepad2.right_bumper && gamepad2.back) {
                if (moCurrentObjective != null) moCurrentObjective.stop();

                moArm_Extend.setPower(0);
                moArm_Twist.setPower(0);
                moArm_PhaseTwo.setPower(0);
                moArm_Extend.setTargetPosition(moArm_Extend.getCurrentPosition());
                moArm_Twist.setTargetPosition(moArm_Twist.getCurrentPosition());
                moArm_PhaseTwo.setTargetPosition(moArm_PhaseTwo.getCurrentPosition());
            }

        // Wave at the viewer!
        // Control = A
            if (canMoveOn() && gamepad2.a && moRuntime.seconds() > miButtonPressDelay) {
                moRuntime.reset();
                moCurrentObjective = foWave;
                foWave.run(moObjectiveRuntime, mbFloor);
            }

        // Grab Sample From Lower Rung
        // Control = B + Dpad_Left
            if (canMoveOn() && gamepad2.b && gamepad2.dpad_left && moRuntime.seconds() > miButtonPressDelay) {
                moRuntime.reset();
                moCurrentObjective = foGrabFromLowerRung;
                foGrabFromLowerRung.run(moObjectiveRuntime, mbFloor);
            }

        // Grab Sample From Floor
        // Control = B + Dpad_Down
            if (canMoveOn() && gamepad2.b && gamepad2.dpad_down && moRuntime.seconds() > miButtonPressDelay) {
                moRuntime.reset();
                moCurrentObjective = foGrabFromFloor;
                foGrabFromFloor.run(moObjectiveRuntime, mbFloor);
            }

        // Return Motors to Zero
        // Control = Back
            if (canMoveOn() && gamepad2.back && moRuntime.seconds() > miButtonPressDelay) {
                moRuntime.reset();
                moCurrentObjective = foZero;
                foZero.run(moObjectiveRuntime, mbFloor);
            }

        // ======================
        // END OBJECTIVE CONTROLS
        // ======================
    }

    private void drivetrainMaster() {
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

        double mdSlowerPowerModifier = 0.5;
        if (mbSlower) {
            mdDrive = mdDrive * mdSlowerPowerModifier;
            mdStrafe = mdStrafe * mdSlowerPowerModifier;
            mdTwist = mdStrafe * mdSlowerPowerModifier;
        }

        double mdSlowPowerModifier = 0.75;
        if (!mbSlow) {
            mdDrive = mdDrive * mdSlowPowerModifier;
            mdStrafe = mdStrafe * mdSlowPowerModifier;
            mdTwist = mdTwist * mdSlowPowerModifier;
        }

        double[] wheelpower = moMecanum.Calculate(mdDrive, mdStrafe, -mdTwist, gamepad2.right_bumper);

        moDrive_FrontLeft.setPower(wheelpower[0]);
        moDrive_FrontRight.setPower(wheelpower[1]);
        moDrive_RearLeft.setPower(wheelpower[2]);
        moDrive_RearRight.setPower(wheelpower[3]);
    }

    @Override
    public void loop() {
        liftarmMaster();
        drivetrainMaster();

        telemetry.addLine("===================================");
        telemetry.addLine("Arm Positions");
        telemetry.addLine("===================================");
        telemetry.addData("Arm Extend Position", moArm_Extend.getCurrentPosition());
        telemetry.addData("Arm Extend Target Position", moArm_Extend.getTargetPosition());
        telemetry.addData("Arm Elbow Position", moArm_PhaseTwo.getCurrentPosition());
        telemetry.addData("Arm Elbow Target Position", moArm_PhaseTwo.getTargetPosition());
        telemetry.addData("Arm Twist Position", moArm_Twist.getCurrentPosition());
        telemetry.addData("Arm Twist Target Position", moArm_Twist.getTargetPosition());
        telemetry.addData("Arm Claw Position", moServoClaw.getPosition());
        telemetry.addLine("===================================");
        telemetry.addLine("");
        telemetry.addLine("===================================");
        telemetry.addLine("DriveTrain");
        telemetry.addLine("===================================");
        telemetry.addData("Front Left Power", moDrive_FrontLeft.getPower());
        telemetry.addData("Front Right Power", moDrive_FrontRight.getPower());
        telemetry.addData("Rear Left Power", moDrive_RearLeft.getPower());
        telemetry.addData("Rear Right Power", moDrive_RearRight.getPower());
        telemetry.addLine("");
        telemetry.addData("Front Left Position", moDrive_FrontLeft.getCurrentPosition());
        telemetry.addData("Front Right Position", moDrive_FrontRight.getCurrentPosition());
        telemetry.addData("Rear Left Position", moDrive_RearLeft.getCurrentPosition());
        telemetry.addData("Rear Right Position", moDrive_RearRight.getCurrentPosition());
        telemetry.addLine("===================================");

        telemetry.update();
    }

}
