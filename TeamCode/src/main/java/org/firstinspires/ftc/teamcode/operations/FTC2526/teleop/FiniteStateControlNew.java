package org.firstinspires.ftc.teamcode.operations.FTC2526.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.IOException;

@TeleOp(name="New State Machine", group="Decode")
public class FiniteStateControlNew extends TeleOPMaster
{
    // STATES
    public enum OperatorState {
        IDLE,
        LOAD_AND_SHOOT,
    }
    enum ShootSubstate {
        IDLE(0.1),
        OPEN_LOADER(0.5),
        CLOSE_LOADER(0.2),
        PUSH(1.2),
        RETRACT(0.2),
        DONE(0.1)
        ;

        public final double duration;

        ShootSubstate(double duration) {
            this.duration = duration;
        }
    }


    //============================================================================================
    // BEGIN VARIABLES
    // EDIT CONTROLS IN controls()
    //============================================================================================
    // SETTINGS
    private final double[]  flywheelMaxVelocities = {3500, 4500, 5500};

    // non-editable
    private int           flywheelPowerIndex = 1;
    private int           numberToShoot = 1;
    private ElapsedTime   timeStep = new ElapsedTime();
    private ElapsedTime   shootStep = new ElapsedTime();

    private OperatorState operatorState = OperatorState.IDLE;
    private OperatorState nextOperatorState = OperatorState.IDLE;
    private ShootSubstate operatorSubState = ShootSubstate.IDLE;

    private int           ballsShot     = 0;
    private boolean enterOperatorState = false;
    //============================================================================================
    // END VARIABLES
    //============================================================================================

    // Throws IOException as some utility classes I wrote require file operations
    public FiniteStateControlNew() throws IOException {
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
        if(cOperator.CANCEL_STATE) {
            operatorState = OperatorState.IDLE;
            nextOperatorState = OperatorState.IDLE;
        }

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
        if (!enterOperatorState) {
            timeStep.reset();
            enterOperatorState = true;
        }

        loaderPosition = LOADER_CLOSED_POSITION;
        pusherPosition = PUSHER_CLOSED_POSITION;

        // Enable/Disable flywheel
        if (cOperator.FLYWHEEL_ON) flywheelVelocity = this.rpmToVelocity(flywheelMaxVelocities[flywheelPowerIndex], FLYWHEEL_COUNTS_PER_REVOLUTION);
        if (cOperator.FLYWHEEL_OFF) flywheelVelocity = FLYWHEEL_MIN_RPM;

        // Increase/Decrease flywheel power
        /*
        if (cOperator.INCREASE_FLYWHEEL_SPEED) flywheelPowerIndex++;
        if (cOperator.DECREASE_FLYWHEEL_SPEED) flywheelPowerIndex--;
        flywheelPowerIndex = Math.max(0, Math.min(flywheelMaxVelocities.length -1, flywheelPowerIndex));
        */

        if (cOperator.INCREASE_FLYWHEEL_SPEED) {
            flywheelPowerIndex = 2;
        }
        else if (cOperator.DECREASE_FLYWHEEL_SPEED) {
            flywheelPowerIndex = 0;
        } else {
            flywheelPowerIndex = 1;
        }

        // Increase/Decrease number of balls to shoot
        if (cOperator.INCREASE_SHOOTCOUNT) numberToShoot++;
        if (cOperator.DECREASE_SHOOTCOUNT) numberToShoot--;
        numberToShoot = Math.max(1, Math.min(3, numberToShoot));

        // Queue load and shoot state
        if (cOperator.LOAD_AND_SHOOT) {
            nextOperatorState = OperatorState.LOAD_AND_SHOOT;
        }

        // If queued state is not the current running state, switch states for next cycle
        if (nextOperatorState != operatorState) {
            operatorState = nextOperatorState;
            enterOperatorState = false;
            return 1;
        }
        return 0;
    }

    public int state_load_and_shoot() {
        int minVelocity = 1980;
        if (!enterOperatorState) {
            timeStep.reset();
            shootStep.reset();
            enterOperatorState = true;

            ballsShot = 0;

            // Always initialize positions
            loaderPosition = LOADER_CLOSED_POSITION;
            pusherPosition = PUSHER_CLOSED_POSITION;
            flywheelVelocity = this.rpmToVelocity(flywheelMaxVelocities[flywheelPowerIndex], FLYWHEEL_COUNTS_PER_REVOLUTION);
        }
        if (ballsShot < numberToShoot) {
            telemetry.addData("Shoot State", operatorSubState.toString());
            telemetry.addData("State Time", shootStep.seconds());

            switch (operatorSubState) {

                case IDLE:
                    // Wait for flywheel to spin up
                    loaderPosition = LOADER_CLOSED_POSITION;
                    pusherPosition = PUSHER_CLOSED_POSITION;

                    if (auxFlywheel.getVelocity() >= minVelocity) {
                        System.out.println("!!! Flywheel meets min velocity!");
                        operatorSubState = ShootSubstate.OPEN_LOADER;
                        shootStep.reset(); // reset timer for OPEN_LOADER
                    }
                    break;

                case OPEN_LOADER:
                    loaderPosition = LOADER_OPEN_POSITION;

                    // Duration of OPEN_LOADER substate
                    if (shootStep.seconds() >= operatorSubState.duration) { // seconds to keep loader open
                        operatorSubState = ShootSubstate.CLOSE_LOADER;
                        shootStep.reset(); // reset timer for CLOSE_LOADER
                    }
                    break;

                case CLOSE_LOADER:
                    loaderPosition = LOADER_CLOSED_POSITION;

                    // Duration of CLOSE_LOADER substate
                    if (shootStep.seconds() >= operatorSubState.duration) {
                        operatorSubState = ShootSubstate.PUSH;
                        shootStep.reset();
                    }
                    break;

                case PUSH:
                    pusherPosition = PUSHER_OPEN_POSITION;

                    // Duration of PUSH substate
                    if (shootStep.seconds() >= operatorSubState.duration) {
                        operatorSubState = ShootSubstate.RETRACT;
                        shootStep.reset();
                    }
                    break;

                case RETRACT:
                    pusherPosition = PUSHER_CLOSED_POSITION;

                    // Duration of RETRACT substate
                    if (shootStep.seconds() >= operatorSubState.duration) {
                        operatorSubState = ShootSubstate.DONE;
                        shootStep.reset();
                    }
                    break;

                case DONE:
                    ballsShot++;
                    operatorSubState = ShootSubstate.IDLE;
                    break;
            }
        }

        // Finished sequence
        if (ballsShot == numberToShoot) {
            loaderPosition = LOADER_CLOSED_POSITION;
            pusherPosition = PUSHER_CLOSED_POSITION;
            flywheelVelocity = FLYWHEEL_MIN_RPM;
            nextOperatorState = OperatorState.IDLE;
        }

        if (nextOperatorState != operatorState) {
            operatorState = nextOperatorState;
            enterOperatorState = false;
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
        double drive = cDriver.DRIVE;
        double strafe = cDriver.STRAFE;
        double twist = cDriver.TWIST;

        strafe = cDriver.DPAD_STRAFE_LEFT  ?  STRAFE_BUTTON_SPEED : strafe;
        strafe = cDriver.DPAD_STRAFE_RIGHT ? -STRAFE_BUTTON_SPEED : strafe;

        drive *= this.driveModifier(cDriver.FULL_SPEED, cDriver.HALF_SPEED, cDriver.GAS, cDriver.BRAKE, DRIVE_BASE_SPEED);
        strafe *= this.driveModifier(cDriver.FULL_SPEED, cDriver.HALF_SPEED, cDriver.GAS, cDriver.BRAKE, STRAFE_BASE_SPEED);
        twist *= this.driveModifier(cDriver.FULL_SPEED, cDriver.HALF_SPEED, cDriver.GAS, cDriver.BRAKE, TWIST_BASE_SPEED);

        double[] wheelPower = mecanum.Calculate(drive, strafe, twist, false);
        //wheelPower = super.tractionControl(wheelPower, DRIVE_SLIP_THRESHOLD);

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
        telemetry.addData("Flywheel Target Velocity", flywheelMaxVelocities[flywheelPowerIndex]);
        telemetry.addData("Balls Shot", ballsShot);
        super.telecom();
    }

    // =============================================================================================
    // END TELEMETRY
    // =============================================================================================

    @Override
    public void controls() {
        cDriver = new ControlState.Builder()
                .drive(this.deadzone(gpDriver.left_stick_y, DEADZONE_STICK))
                .strafe(this.deadzone(-gpDriver.left_stick_x, DEADZONE_STICK))
                .twist(this.deadzone(-gpDriver.right_stick_x, DEADZONE_STICK))
                .halfSpeed(gpDriver.left_bumper || gpDriver.dpad_down)
                .fullSpeed(gpDriver.right_bumper || gpDriver.dpad_up)
                .gas(gpDriver.a ? 1 : gpDriver.right_trigger)
                .brake(gpDriver.y ? 1 : gpDriver.left_trigger)
                .dpadLeft(gpDriver.dpad_left)
                .dpadRight(gpDriver.dpad_right)
                .build();

        cOperator = new ControlState.Builder()
                .operatorDrive(this.deadzone(gpOperator.right_stick_y, DEADZONE_STICK) * OPERATOR_DRIVE_MODIFIER)
                .operatorStrafe(this.deadzone(-gpOperator.left_stick_x, DEADZONE_STICK) * OPERATOR_DRIVE_MODIFIER)
                .operatorTwist(this.deadzone(-gpOperator.right_stick_x, DEADZONE_STICK) * OPERATOR_DRIVE_MODIFIER)
                .loadAndShoot(gpOperator.right_bumper|| gpOperator.cross || gpOperator.a || Math.abs(gpOperator.right_trigger) > DEADZONE_TRIGGER_BUTTON)
                .flywheelOn(gpOperator.dpad_up || gpOperator.touchpad)
                .flywheelOff(gpOperator.dpad_down)
                .cancelState(gpOperator.back)
                .decreaseShootcount(gpOperator.dpad_left)
                .increaseShootcount(gpOperator.dpad_right)
                .increaseFlywheelSpeed(gpOperator.square || gpOperator.x)
                .decreaseFlywheelSpeed(gpOperator.circle || gpOperator.b)
                .build();
    }

    @Override
    public void loop() {
        super.loop();
    }
}