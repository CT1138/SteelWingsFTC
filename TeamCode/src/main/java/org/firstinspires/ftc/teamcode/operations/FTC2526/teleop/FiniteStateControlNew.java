package org.firstinspires.ftc.teamcode.operations.FTC2526.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.IOException;

@TeleOp(name="New State Machine", group="Decode")
public class FiniteStateControlNew extends TeleOPMaster
{
    // STATES
    enum OperatorState {
        IDLE,
        LOAD_AND_SHOOT,
    }
    enum ShootSubstate {
        IDLE(0.0),
        OPEN_LOADER(0.0),
        CLOSE_LOADER(0.3),
        PUSH(0.4),
        RETRACT(1),
        DONE(0)
        ;

        public final double startTime;

        ShootSubstate(double startTime) {
            this.startTime = startTime;
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
    private OperatorState nextState = OperatorState.IDLE;
    private ShootSubstate subState = ShootSubstate.IDLE;

    private int           ballsShot     = 0;
    private boolean       enterState = false;
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
            nextState = OperatorState.IDLE;
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
        if (!enterState) {
            timeStep.reset();
            enterState = true;
        }

        loaderPosition = LOADER_CLOSED_POSITION;
        pusherPosition = PUSHER_CLOSED_POSITION;

        // Enable/Disable flywheel
        if (cOperator.FLYWHEEL_ON) flywheelVelocity = this.rpmToVelocity(flywheelMaxVelocities[flywheelPowerIndex], FLYWHEEL_COUNTS_PER_REVOLUTION);
        if (cOperator.FLYWHEEL_OFF) flywheelVelocity = FLYWHEEL_MIN_RPM;

        // Increase/Decrease flywheel power
        if (cOperator.INCREASE_FLYWHEEL_SPEED) flywheelPowerIndex++;
        if (cOperator.DECREASE_FLYWHEEL_SPEED) flywheelPowerIndex--;
        flywheelPowerIndex = Math.max(0, Math.min(flywheelMaxVelocities.length -1, flywheelPowerIndex));

        // Increase/Decrease number of balls to shoot
        if (cOperator.INCREASE_SHOOTCOUNT) numberToShoot++;
        if (cOperator.DECREASE_SHOOTCOUNT) numberToShoot--;
        numberToShoot = Math.max(1, Math.min(3, numberToShoot));

        // Queue load and shoot state
        if (cOperator.LOAD_AND_SHOOT) {
            nextState = OperatorState.LOAD_AND_SHOOT;
        }

        // If queued state is not the current running state, switch states for next cycle
        if (nextState != operatorState) {
            operatorState = nextState;
            enterState = false;
            return 1;
        }
        return 0;
    }

    private int state_load_and_shoot() {
        int minVelocity = 1980;
        double time = shootStep.seconds();

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
            telemetry.addData("Shoot State", subState.toString());
            telemetry.addData("Shooter Time", time);
            telemetry.addData("State Time", timeStep.seconds());

            switch (subState) {
                case IDLE:
                    if(auxFlywheel.getVelocity() < minVelocity) {
                        System.out.println("!!! Flywheel under minimum velocity, waiting . . .");
                        shootStep.reset();
                        loaderPosition = LOADER_CLOSED_POSITION;
                        pusherPosition = PUSHER_CLOSED_POSITION;
                    } else {
                        System.out.println("!!! Flywheel meets min velocity!");
                        subState = ShootSubstate.OPEN_LOADER;
                    }
                    break;

                case OPEN_LOADER:
                    loaderPosition = LOADER_OPEN_POSITION;
                    if (subState.startTime <= time) {
                        subState = ShootSubstate.CLOSE_LOADER;
                        shootStep.reset();
                    }
                    break;

                case CLOSE_LOADER:
                    loaderPosition = LOADER_CLOSED_POSITION;
                    if (subState.startTime <= time) {
                        subState = ShootSubstate.PUSH;
                        shootStep.reset();
                    }
                    break;

                case PUSH:
                    pusherPosition = PUSHER_OPEN_POSITION;
                    if (subState.startTime <= time) {
                        subState = ShootSubstate.RETRACT;
                        shootStep.reset();
                    }
                    break;

                case RETRACT:
                    pusherPosition = PUSHER_CLOSED_POSITION;
                    if (subState.startTime <= time) {
                        subState = ShootSubstate.DONE;
                        shootStep.reset();
                    }
                    break;

                case DONE:
                    ballsShot++;
                    subState = ShootSubstate.IDLE;
                    break;
            }
        }

        // Finished sequence
        if (ballsShot == numberToShoot) {
            loaderPosition = LOADER_CLOSED_POSITION;
            pusherPosition = PUSHER_CLOSED_POSITION;
            flywheelVelocity = FLYWHEEL_MIN_RPM;
            nextState = OperatorState.IDLE;
        }

        if (nextState != operatorState) {
            operatorState = nextState;
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