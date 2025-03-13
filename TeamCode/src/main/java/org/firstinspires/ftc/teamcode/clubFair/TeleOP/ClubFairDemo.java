package org.firstinspires.ftc.teamcode.clubFair.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.clubFair.FairObjectives;
import org.firstinspires.ftc.teamcode.core.Objective;

@TeleOp(name="Club Fair Demo", group="Into-The-Deep")
public class ClubFairDemo extends OpMode
{
    // ===============
    // BEGIN VARIABLES
    // ===============
        // Robot Hardware Setup
            private DcMotor Lift;
            private DcMotor Elbow;
            private DcMotor Wrist;
            private Servo Claw;
            private TouchSensor Touch = null;
        // Runtimes
            private final ElapsedTime Runtime = new ElapsedTime();
            private final ElapsedTime ObjectiveRuntime = new ElapsedTime();
        // Objectives
            private Objective moCurrentObjective = null;
            private FairObjectives moFairObjectives;
        // Utilities

        // Robot Hardware Data
            private DcMotor[] Motors;
            private Servo[] Servos;
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

            Lift = hardwareMap.get(DcMotor.class, "Lift");
            Elbow = hardwareMap.get(DcMotor.class, "Elbow");
            Wrist = hardwareMap.get(DcMotor.class, "Wrist");

            Claw = hardwareMap.get(Servo.class, "Claw");

            // Set Direction for Lift different than the rest
            Lift.setDirection(DcMotorSimple.Direction.REVERSE);

            // Build motor array
            Motors = new DcMotor[]{ Lift, Elbow, Wrist };
            Servos = new Servo[]{ Claw };

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

                    for (DcMotor motor : Motors) {
                        motor.setPower(0);
                        motor.setTargetPosition(motor.getCurrentPosition());
                    }
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

        @Override
        public void loop() {
            liftarmMaster();

            telemetry.update();
        }

    // ===============
    // END METHODS
    // ===============
}
