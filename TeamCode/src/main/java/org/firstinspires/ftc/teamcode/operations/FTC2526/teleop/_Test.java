package org.firstinspires.ftc.teamcode.operations.FTC2526.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.IOException;

@Disabled
@TeleOp(name="TestOp", group="Decode")
public class _Test extends TeleOPMaster
{
    //============================================================================================
    // BEGIN VARIABLES
    // EDIT CONTROLS IN loop()
    //============================================================================================
    // SETTINGS
        // CONSTANTS
        private final double    DEADZONE_TRIGGER_BUTTON = 0.5;
        private final double    DEADZONE_STICK = 0.05;
        private final double    OPERATOR_DRIVE_MODIFIER = 0.5;
        private final double    DRIVE_BASE_SPEED = 0.8;
        private final double    STRAFE_BUTTON_SPEED = 0.15;
        private final double    TWIST_BASE_SPEED = 0.5;
        private final double    STRAFE_BASE_SPEED = 0.5;

        private final int       DRIVE_MAX_RPM = 5500;
        private final double    DRIVE_SLIP_THRESHOLD = 1.2;

        private final int       FLYWHEEL_MIN_RPM = 0;
        private final int       FLYWHEEL_MAX_RPM = 5500;

        private final double PUSHER_OPEN_POSITION = 0.9;
        private final double PUSHER_CLOSED_POSITION = 0.52;

        private final double    LOADER_OPEN_POSITION = 0.3;
        private final double    LOADER_CLOSED_POSITION = 0.5;

    private double    CONTROL_DRIVE;
    private double    CONTROL_STRAFE;
    private double    CONTROL_TWIST;
    private boolean   CONTROL_HALF_SPEED;
    private boolean   CONTROL_FULL_SPEED;
    private double    CONTROL_GAS;
    private double    CONTROL_BRAKE;
    private boolean   CONTROL_DPAD_STRAFE_LEFT;
    private boolean   CONTROL_DPAD_STRAFE_RIGHT;

    private boolean   CONTROL_FLYWHEEL;
    private boolean CONTROL_PUSHER;
    private boolean   CONTROL_LOADER;
    private double    CONTROL_OPERATOR_DRIVE;
    private double    CONTROL_OPERATOR_STRAFE;
    private double    CONTROL_OPERATOR_TWIST;

    //============================================================================================
    // END VARIABLES
    //============================================================================================

    // Throws IOException as some utility classes I wrote require file operations
    public _Test() throws IOException {
    }

    // =============================================================================================
    // END HELPER METHODS
    // =============================================================================================

    // Prep our motors and servos
    @Override
    public void init() {
        super.init();
        auxPusher.setPosition(LOADER_CLOSED_POSITION);
        auxLoader.setPosition(PUSHER_CLOSED_POSITION);
    }

    // Reset our timers
    @Override
    public void start() {
        super.start();
    }

    // =============================================================================================
    // OPERATOR METHODS
    // =============================================================================================
    @Override
    public void operator() {
        // Initial Powers
        double flywheelVelocity = gamepad2.right_stick_y;
        double pusherPosition = gamepad2.right_trigger;
        double loaderPosition = gamepad2.left_trigger;
        double intakeVelocity = 0;

        // Apply outputs
        auxIntake.setVelocity(intakeVelocity);
        auxPusher.setPosition(pusherPosition);
        auxLoader.setPosition(loaderPosition);
        auxFlywheel.setVelocity(flywheelVelocity);
    }
    // =============================================================================================
    // END OPERATOR METHODS
    // =============================================================================================

    // =============================================================================================
    // DRIVER METHODS
    // =============================================================================================
    @Override
    public void driver() {
        double drive = CONTROL_DRIVE;
        double strafe = CONTROL_STRAFE;
        double twist = CONTROL_TWIST;

        drive = CONTROL_DPAD_STRAFE_LEFT  ? -STRAFE_BUTTON_SPEED : drive;
        drive = CONTROL_DPAD_STRAFE_RIGHT ?  STRAFE_BUTTON_SPEED : drive;

        drive *= this.driveModifier(CONTROL_FULL_SPEED, CONTROL_HALF_SPEED, CONTROL_GAS, CONTROL_BRAKE, DRIVE_BASE_SPEED);
        strafe *= this.driveModifier(CONTROL_FULL_SPEED, CONTROL_HALF_SPEED, CONTROL_GAS, CONTROL_BRAKE, STRAFE_BASE_SPEED);
        twist *= this.driveModifier(CONTROL_FULL_SPEED, CONTROL_HALF_SPEED, CONTROL_GAS, CONTROL_BRAKE, TWIST_BASE_SPEED);

        double[] wheelPower = this.tractionControl(mecanum.Calculate(drive, strafe, twist), DRIVE_SLIP_THRESHOLD);

        double maxVelocity = this.rpmToVelocity(DRIVE_MAX_RPM, DRIVE_COUNTS_PER_REVOLUTION);
        driveFL.setVelocity(wheelPower[0] * maxVelocity);
        driveFR.setVelocity(wheelPower[1] * maxVelocity);
        driveRL.setVelocity(wheelPower[2] * maxVelocity);
        driveRR.setVelocity(wheelPower[3] * maxVelocity);
    }
    // =============================================================================================
    // END DRIVER METHODS
    // =============================================================================================


    // =============================================================================================
    // TELEMETRY
    // =============================================================================================
    @Override
    public void telecom() {
        super.telecom();
    }

    // =============================================================================================
    // END TELEMETRY
    // =============================================================================================

    @Override
    public void loop() {
    // =============================================================================================
    // CONTROLS
    // =============================================================================================
        // DRIVER
        CONTROL_DRIVE  = this.deadzone(gpDriver.right_stick_y, DEADZONE_STICK);
        CONTROL_STRAFE = this.deadzone(-gpDriver.left_stick_x, DEADZONE_STICK);
        CONTROL_TWIST  = this.deadzone(-gpDriver.right_stick_x, DEADZONE_STICK);
        CONTROL_FULL_SPEED = gpDriver.right_bumper || gpDriver.dpad_up;
        CONTROL_HALF_SPEED = gpDriver.left_bumper || gpDriver.dpad_down;
        CONTROL_GAS   = gpDriver.a ? 1 : gpDriver.right_trigger;
        CONTROL_BRAKE = gpDriver.y ? 1 : gpDriver.left_trigger;
        CONTROL_DPAD_STRAFE_LEFT = gpDriver.dpad_left;
        CONTROL_DPAD_STRAFE_RIGHT = gpDriver.dpad_right;


        // OPERATOR
        CONTROL_OPERATOR_DRIVE = this.deadzone(gpOperator.right_stick_y, DEADZONE_STICK) * OPERATOR_DRIVE_MODIFIER;
        CONTROL_OPERATOR_STRAFE = this.deadzone(-gpOperator.left_stick_x, DEADZONE_STICK) * OPERATOR_DRIVE_MODIFIER;
        CONTROL_OPERATOR_TWIST = this.deadzone(-gpOperator.right_stick_x, DEADZONE_STICK) * OPERATOR_DRIVE_MODIFIER;
        CONTROL_FLYWHEEL = gpOperator.right_bumper
                            || gpOperator.y
                            || Math.abs(gpOperator.right_trigger) > DEADZONE_TRIGGER_BUTTON
        ;
        CONTROL_PUSHER = gpOperator.dpad_right || gpOperator.x;
        CONTROL_LOADER = gpOperator.dpad_left || gpOperator.b;

        //===================
        // LEAVE THESE ALONE
        //===================
        CONTROL_DRIVE = Math.abs(CONTROL_DRIVE) > 0 ? CONTROL_DRIVE : CONTROL_OPERATOR_DRIVE;
        CONTROL_STRAFE = Math.abs(CONTROL_STRAFE) > 0 ? CONTROL_STRAFE : CONTROL_OPERATOR_STRAFE;
        CONTROL_TWIST = Math.abs(CONTROL_TWIST) > 0 ? CONTROL_TWIST : CONTROL_OPERATOR_TWIST;
        CONTROL_DRIVE = Math.max(-1, Math.min(1, CONTROL_DRIVE));
        CONTROL_STRAFE = Math.max(-1, Math.min(1, CONTROL_STRAFE));
        CONTROL_TWIST = Math.max(-1, Math.min(1, CONTROL_TWIST));

        // Run last
        super.loop();
    }
}