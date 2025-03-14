package org.firstinspires.ftc.teamcode.ClubFair25.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Club Fair Manual", group="Into-The-Deep")
public class ClubFairDemoManual extends OpMode
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
            Elbow.setDirection(DcMotor.Direction.REVERSE);
            Wrist.setDirection(DcMotor.Direction.REVERSE);


            // Build motor array
            Motors = new DcMotor[]{ Lift, Elbow, Wrist };
            Servos = new Servo[]{ Claw };

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
        private void liftarm() {
            if(gamepad2.y) {
                changeMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                changeMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            double mdBrake = ( -gamepad2.right_trigger + 1 );
            double mdGas = ( gamepad2.right_trigger + 1 );

            // Arm Lift power
            double mdElbowPower = 0;
            mdElbowPower = -gamepad2.right_stick_y;
            if(!gamepad2.left_bumper) mdElbowPower = mdElbowPower * 0.5;
            Elbow.setPower(mdElbowPower);

            // Arm extension
            double mdExtendPower = gamepad2.left_stick_y;
            if (Touch.isPressed() && gamepad2.left_stick_y > 0) {
                mdExtendPower = 0;
            }
            Lift.setPower(mdExtendPower);

            // WRIST CONTROLS
            if (gamepad2.right_bumper) {
                // If bumper is pressed, lift speed cap slightly and allow override via trigger press
                Wrist.setPower(gamepad2.right_stick_x / 2 * mdGas);
            } else {
                // If bumper is not pressed, keep speed cap at 25% and allow braking via trigger press
                Wrist.setPower(gamepad2.right_stick_x / 4 * mdBrake);
            }

            // CLAW DPAD CONTROLS.... DEPRECATED???
            /*
            if (gamepad2.dpad_up || gamepad2.dpad_down) {
                if (gamepad2.dpad_up) {
                    Claw.setPosition(-1);
                }

                if (gamepad2.dpad_down) {
                    Claw.setPosition(1);
                }
            }
             */

            // CLAW TRIGGER CONTROL
            Claw.setPosition(1 - gamepad2.left_trigger);


            // TELEMETRY
            telemetry.addLine("===================================");
            telemetry.addLine("Arm Positions");
            telemetry.addLine("===================================");
            telemetry.addData("Arm Extend Position", Lift.getCurrentPosition());
            telemetry.addData("Arm Elbow Position", Elbow.getCurrentPosition());
            telemetry.addData("Arm Twist Position", Wrist.getCurrentPosition());
            telemetry.addData("Arm Claw Position", Claw.getPosition());
            telemetry.addLine("===================================");
        }

        private void changeMotorMode(DcMotor.RunMode mode) {
            for (DcMotor motor : Motors) {
                motor.setMode(mode);
            }
        }

        @Override
        public void loop() {
            liftarm();

            telemetry.update();
        }

    // ===============
    // END METHODS
    // ===============
}
