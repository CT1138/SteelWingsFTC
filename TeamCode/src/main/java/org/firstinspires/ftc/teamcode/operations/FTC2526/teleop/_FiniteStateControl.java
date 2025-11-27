package org.firstinspires.ftc.teamcode.operations.FTC2526.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.IOException;

@Disabled
@TeleOp(name="Finite State Machine", group="Decode")
public class _FiniteStateControl extends TeleOPMaster
{
    // STATES
    enum OperatorState {
        IDLE,
        LOAD_AND_SHOOT,
    }

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

    private double    flywheelVelocity;
    private double    loaderPosition = LOADER_CLOSED_POSITION;
    private double    pusherPosition = PUSHER_CLOSED_POSITION;
    private double[]  flywheelMaxVelocities = {FLYWHEEL_MAX_RPM, FLYWHEEL_MAX_RPM - 1000, FLYWHEEL_MAX_RPM - 2000};

    private double    CONTROL_DRIVE;
    private double    CONTROL_STRAFE;
    private double    CONTROL_TWIST;
    private boolean   CONTROL_HALF_SPEED;
    private boolean   CONTROL_FULL_SPEED;
    private double    CONTROL_GAS;
    private double    CONTROL_BRAKE;
    private boolean   CONTROL_DPAD_STRAFE_LEFT;
    private boolean   CONTROL_DPAD_STRAFE_RIGHT;

    private boolean CONTROL_LOAD_AND_SHOOT;
    private boolean   CONTROL_INCREASE_SHOOTCOUNT;
    private boolean   CONTROL_DECREASE_SHOOTCOUNT;
    private boolean CONTROL_SHOOT;
    private boolean CONTROL_LOAD;
    private boolean   CONTROL_PUSHER;
    private double    CONTROL_OPERATOR_DRIVE;
    private double    CONTROL_OPERATOR_STRAFE;
    private double    CONTROL_OPERATOR_TWIST;
    private boolean CONTROL_FLYWHEEL_ON;
    private boolean CONTROL_FLYWHEEL_OFF;
    private boolean CONTROL_INCREASE_SPEED;
    private boolean CONTROL_DECREASE_SPEED;

    private ElapsedTime   timeStep = new ElapsedTime();
    private ElapsedTime   shootStep = new ElapsedTime();

    private OperatorState operatorState = OperatorState.IDLE;
    private OperatorState newState = OperatorState.IDLE;
    private int           flywheelPowerIndex = 1;
    private int           numberToShoot = 1;
    private int           ballsShot     = 0;
    private boolean       enterState = false;

    //============================================================================================
    // END VARIABLES
    //============================================================================================

    // Throws IOException as some utility classes I wrote require file operations
    public _FiniteStateControl() throws IOException {
    }

    // =============================================================================================
    // END HELPER METHODS
    // =============================================================================================

    // Prep our motors and servos
    @Override
    public void init() {
        super.init();
        auxPusher.setPosition(PUSHER_CLOSED_POSITION);
        auxLoader.setPosition(LOADER_CLOSED_POSITION);
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
        // Set LED color on controller to signal the current # to shoot
        int[] ledColor = {0, 0, 0};
        switch (numberToShoot) {
            case 1:
                ledColor = new int[]{222, 218, 1000};
                break;
            case 2:
                ledColor = new int[]{222, 173, 1000};
                break;
            case 3:
                ledColor = new int[]{222, 100, 1000};
                break;
        }
        gamepad2.setLedColor(ledColor[0], ledColor[1], ledColor[2], 10);

        // State handler
        switch (operatorState) {
            case IDLE:
                state_idle();
                break;
            case LOAD_AND_SHOOT:
                state_load_and_shoot();
                break;
        }

        // Apply positions and powers post-state
        auxPusher.setPosition(pusherPosition);
        auxLoader.setPosition(loaderPosition);
        auxFlywheel.setVelocity(flywheelVelocity);
    }

    // Default state
    private int state_idle() {
        if (!enterState) {
            timeStep.reset();
            enterState = true;
        }

        // Enable/Disable flywheel
        if (CONTROL_FLYWHEEL_ON) flywheelVelocity = this.rpmToVelocity(flywheelMaxVelocities[flywheelPowerIndex], FLYWHEEL_COUNTS_PER_REVOLUTION);
        if (CONTROL_FLYWHEEL_OFF) flywheelVelocity = FLYWHEEL_MIN_RPM;

        // Increase/Decrease flywheel power
        if (CONTROL_INCREASE_SPEED) flywheelPowerIndex++;
        if (CONTROL_DECREASE_SPEED) flywheelPowerIndex--;
        flywheelPowerIndex = Math.max(0, Math.min(2, flywheelPowerIndex));
        // Increase/Decrease number of balls to shoot
        if (CONTROL_INCREASE_SHOOTCOUNT) numberToShoot++;
        if (CONTROL_DECREASE_SHOOTCOUNT) numberToShoot--;
        numberToShoot = Math.max(1, Math.min(3, numberToShoot));

        // Queue load and shoot state
        if (CONTROL_LOAD_AND_SHOOT) {
            newState = OperatorState.LOAD_AND_SHOOT;
        }

        // If queued state is not the current running state, switch states for next cycle
        if (newState != operatorState) {
            operatorState = newState;
            enterState = false;
            return 1;
        }
        return 0;
    }

    private int state_load_and_shoot() {
        if (!enterState) {
            timeStep.reset();
            shootStep.reset();
            enterState = true;

            ballsShot = 0;

            // Always initialize positions
            loaderPosition = LOADER_CLOSED_POSITION;
            pusherPosition = PUSHER_CLOSED_POSITION;
            flywheelVelocity = this.rpmToVelocity(flywheelMaxVelocities[flywheelPowerIndex], FLYWHEEL_COUNTS_PER_REVOLUTION);
        }

        if (ballsShot < numberToShoot) {

            // 1. Open loader briefly
            if (shootStep.seconds() < 0.2) {
                loaderPosition = LOADER_OPEN_POSITION;
            }

            // 2. Close loader
            if (shootStep.seconds() > 0.3) {
                loaderPosition = LOADER_CLOSED_POSITION;
            }

            // 3. Push ball
            if (shootStep.seconds() > 0.6) {
                pusherPosition = PUSHER_OPEN_POSITION;
            }

            // 4. Retract pusher after adequate time
            if (shootStep.seconds() > 1) {
                pusherPosition = PUSHER_CLOSED_POSITION;

                ballsShot++;
                shootStep.reset();
            }
        }

        // Finished sequence
        if (ballsShot == numberToShoot) {
            loaderPosition = LOADER_CLOSED_POSITION;
            pusherPosition = PUSHER_CLOSED_POSITION;
            flywheelVelocity = FLYWHEEL_MIN_RPM;
            newState = OperatorState.IDLE;
        }

        if (newState != operatorState) {
            operatorState = newState;
            enterState = false;
            return 1;
        }
        return 0;
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
        telemetry.addData("State", operatorState.toString());
        telemetry.addData("Number to Shoot", numberToShoot);
        telemetry.addData("Balls Shot", ballsShot);
        telemetry.addData("Shoot Step (seconds)", shootStep.seconds());
        telemetry.addData("Flywheel Target Velocity", flywheelMaxVelocities[flywheelPowerIndex]);
        telemetry.addData("Time Step (seconds)", timeStep.seconds());
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
        CONTROL_DRIVE  = this.deadzone(gpDriver.left_stick_y, DEADZONE_STICK);
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
        CONTROL_LOAD_AND_SHOOT = gpOperator.right_bumper
                            || gpOperator.a
                            || Math.abs(gpOperator.right_trigger) > DEADZONE_TRIGGER_BUTTON
        ;
        CONTROL_FLYWHEEL_ON = gpOperator.dpad_up;
        CONTROL_FLYWHEEL_OFF = gpOperator.dpad_down;
        CONTROL_SHOOT = false;
        CONTROL_LOAD = false;
        CONTROL_PUSHER = false;
        CONTROL_DECREASE_SHOOTCOUNT = gpOperator.dpad_left;
        CONTROL_INCREASE_SHOOTCOUNT = gpOperator.dpad_right;
        CONTROL_INCREASE_SPEED = gpOperator.x;
        CONTROL_DECREASE_SPEED = gpOperator.b;


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