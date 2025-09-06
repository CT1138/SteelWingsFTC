package org.firstinspires.ftc.teamcode.operations.FTC2425.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.operations.ClubFair25.FairObjectives;

import org.firstinspires.ftc.teamcode.core.Objective;
import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.core.util.Mecanum;
import org.firstinspires.ftc.teamcode.core.util.MotorUtilities;

@TeleOp(name="Assisted Driver", group="Into-The-Deep")
@Disabled
public class AssistedDriver extends OpMode
{
    // ===============
    // BEGIN VARIABLES
    // ===============
        // Robot Hardware Setup
            private final Robot myRobot = new Robot(
                    hardwareMap,
                    new String[]{ "Drive_FrontLeft", "Drive_FrontRight", "Drive_RearLeft", "Drive_RearRight", "Arm_Extend", "Arm_PhaseTwo", "Arm_Twist"},
                    new String[]{ "Servo_Claw" }
            );
        // Parameters
            private final double mdDefaultSpeed = 0.75;
            private final double mdSlowSpeed = 0.75;
            private final double mdSlowerSpeed = 0.5;
        // Runtimes
            private final ElapsedTime Runtime = new ElapsedTime();
            private final ElapsedTime ObjectiveRuntime = new ElapsedTime();
        // Objectives
            private Objective moCurrentObjective = null;
            private FairObjectives moFairObjectives;
        // Utilities
            private final Mecanum UtilMecanum = new Mecanum(mdDefaultSpeed);
            MotorUtilities UtilMotors = new MotorUtilities(myRobot);
        // Robot Hardware
            private DcMotor[] Motors;
            private String[] msDriveMotors;
            private String[] msArmMotors;
            private Servo[] Servos;
            private Servo Claw = null;
            private TouchSensor Touch = null;
    // ===============
    // END VARIABLES
    // ===============

    // ===============
    // BEGIN METHODS
    // ===============
        @Override
        public void init() {
            // Single execution on INIT

            // Define Robot
            Touch = hardwareMap.get(TouchSensor.class, "TouchSensor");

            msDriveMotors = new String[]{ "Drive_FrontLeft", "Drive_FrontRight", "Drive_RearLeft", "Drive_RearRight" };
            UtilMotors.batchSetup(msDriveMotors, "forward", "run using encoder", "brake");

            msArmMotors = new String[]{ "Arm_Extend", "Arm_PhaseTwo", "Arm_Twist" };
            UtilMotors.batchSetup(msArmMotors, "", "run using encoder", "brake");

            // Set Direction for Arm_Extend different than the rest
            myRobot.motor("Arm_Extend").setDirection(DcMotorSimple.Direction.REVERSE);

            // Build motor array
            Motors = myRobot.getMotorArray();
            Servos = myRobot.getServoArray();

            // Get objectives
            moFairObjectives = new FairObjectives(Motors, Servos);
        }
        @Override
        public void start() {
            ObjectiveRuntime.reset();
            Runtime.reset();
            Claw.setPosition(1);
        }
        private boolean canMoveOn() {
            for (DcMotor motor : Motors) if (motor.isBusy() && motor.getCurrentPosition() != motor.getTargetPosition()) return false;
            return true;
        }
        private void liftarmMaster() {
                boolean mbFloor = Touch.isPressed();
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

                    UtilMotors.freeze(msArmMotors);
                }

            // Wave at the viewer!
            // Control = A
                if (canMoveOn() && gamepad2.a && Runtime.seconds() > miButtonPressDelay) {
                    Runtime.reset();
                    moCurrentObjective = foWave;
                    foWave.run(ObjectiveRuntime, mbFloor);
                }

            // Grab Sample From Lower Rung
            // Control = B + Dpad_Left
                if (canMoveOn() && gamepad2.b && gamepad2.dpad_left && Runtime.seconds() > miButtonPressDelay) {
                    Runtime.reset();
                    moCurrentObjective = foGrabFromLowerRung;
                    foGrabFromLowerRung.run(ObjectiveRuntime, mbFloor);
                }

            // Grab Sample From Floor
            // Control = B + Dpad_Down
                if (canMoveOn() && gamepad2.b && gamepad2.dpad_down && Runtime.seconds() > miButtonPressDelay) {
                    Runtime.reset();
                    moCurrentObjective = foGrabFromFloor;
                    foGrabFromFloor.run(ObjectiveRuntime, mbFloor);
                }

            // Return Motors to Zero
            // Control = Back
                if (canMoveOn() && gamepad2.back && Runtime.seconds() > miButtonPressDelay) {
                    Runtime.reset();
                    moCurrentObjective = foZero;
                    foZero.run(ObjectiveRuntime, mbFloor);
                }

            // ======================
            // END OBJECTIVE CONTROLS
            // ======================
        }
        private void drivetrainMaster() {
            // Drive Controls
                double mdDrive = gamepad1.left_stick_y;
                double mdStrafe = -gamepad1.left_stick_x;
                double mdTwist = gamepad1.right_stick_x;

            // Modifier Controls
                boolean mbSlow = gamepad1.right_bumper;
                boolean mbSlower = gamepad1.left_bumper;
                double mdBrake = gamepad1.right_trigger;

            // Simple Controls:
                // DPAD Controls to move robot in one direction
                    if (gamepad1.dpad_left)
                        mdStrafe = 1 - mdBrake;
                    if (gamepad1.dpad_right)
                        mdStrafe = -1 + mdBrake;
                    if (gamepad1.dpad_up)
                        mdDrive = -1 + mdBrake;
                    if (gamepad1.dpad_down)
                        mdDrive = 1 - mdBrake;

                // X and B controls to turn robot
                    if (gamepad1.x) mdTwist = -1 + mdBrake;
                    if (gamepad1.b) mdTwist = 1 - mdBrake;

            // Modifiers
                // Slow/Slower
                    if (mbSlower) {
                        mdDrive *= mdSlowerSpeed;
                        mdStrafe *= mdSlowerSpeed;
                        mdTwist *= mdSlowerSpeed;
                    }
                    if (mbSlow) {
                        mdDrive *= mdSlowSpeed;
                        mdStrafe *= mdSlowSpeed;
                        mdTwist *= mdSlowSpeed;
                    }

                // Brakes
                    mdDrive -= mdBrake;
                    mdStrafe -= mdBrake;
                    mdTwist -= mdBrake;

            // Calculate Drive Power and Assign
                double[] wheelpower = UtilMecanum.Calculate(mdDrive, mdStrafe, -mdTwist, gamepad2.right_bumper);

                for (int i = 0; i < msDriveMotors.length; i++) {
                    myRobot.motor(msDriveMotors[i]).setPower(wheelpower[i]);
                }
        }
        @Override
        public void loop() {
            liftarmMaster();
            drivetrainMaster();

            telemetry.update();
        }
    // ===============
    // END METHODS
    // ===============
}
