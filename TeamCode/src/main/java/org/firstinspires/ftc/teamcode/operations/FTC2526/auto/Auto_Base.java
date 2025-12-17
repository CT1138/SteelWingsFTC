package org.firstinspires.ftc.teamcode.operations.FTC2526.auto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.core.util.Mecanum;

@Disabled
@Autonomous(name="Auto - Base", group="Decode")
public class Auto_Base extends OpMode {
    //============================================================================================
    // BEGIN VARIABLES
    // EDIT CONTROLS IN loop()
    //============================================================================================
    // SETTINGS
    // CONSTANTS
    enum OperatorState {
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

    public double[]  flywheelMaxVelocities = {3500, 4500, 5500};

    // non-editable
    public int           flywheelPowerIndex = 1;
    public int           numberToShoot = 1;
    public ElapsedTime   timeStep = new ElapsedTime();
    public ElapsedTime   shootStep = new ElapsedTime();

    public OperatorState operatorState = OperatorState.IDLE;
    public OperatorState nextOperatorState = OperatorState.IDLE;
    public ShootSubstate operatorSubState = ShootSubstate.IDLE;

    public int           ballsShot     = 0;
    public boolean enterOperatorState = false;
    public boolean cancelOperatorState = false;
    public boolean doneFiring = false;
    public final double    DRIVE_BASE_SPEED = 0.8;
    public final double    TWIST_BASE_SPEED = 0.5;
    public final double    STRAFE_BASE_SPEED = 0.5;
    public final double TRACK_WIDTH_ERROR = (90 / 37);
    public final double    TRACK_WIDTH_INCHES = 14.25 * TRACK_WIDTH_ERROR;

    public final int       DRIVE_COUNTS_PER_REVOLUTION = 28;
    public final int       FLYWHEEL_COUNTS_PER_REVOLUTION = 28;
    public final int       INTAKE_COUNTS_PER_REVOLUTION = 28;
    public final double DRIVE_GEAR_REDUCTION = 12;   // gear ratio
    public final double WHEEL_DIAMETER_INCHES = 2.95275591; // wheel size
    public final double COUNTS_PER_INCH_ERROR = 1 - 0.182;
    public final double COUNTS_PER_INCH = ((DRIVE_COUNTS_PER_REVOLUTION * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI)) * COUNTS_PER_INCH_ERROR; //3.1415);

    public final int       DRIVE_MAX_RPM = 5500;
    public final double    DRIVE_SLIP_THRESHOLD = 1.2;

    public final int       FLYWHEEL_MIN_RPM = 0;
    public final int       FLYWHEEL_MAX_RPM = 5500;

    public final double PUSHER_OPEN_POSITION = 0.9;
    public final double PUSHER_CLOSED_POSITION = 0.52;

    public final double    LOADER_OPEN_POSITION = 0.3;
    public final double    LOADER_CLOSED_POSITION = 0.45;

    // =============================================================================================
    public double    flywheelVelocity;
    public double    loaderPosition = LOADER_CLOSED_POSITION;
    public double    pusherPosition = PUSHER_CLOSED_POSITION;
    public final RevHubOrientationOnRobot.LogoFacingDirection CONTROL_LOGO_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    public final RevHubOrientationOnRobot.UsbFacingDirection CONTROL_USB_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.UP;
    public final RevHubOrientationOnRobot.LogoFacingDirection EXPANSION_LOGO_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
    public final RevHubOrientationOnRobot.UsbFacingDirection EXPANSION_USB_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;
    // END CONSTANTS

    // PIDF Settings:
    // (P)roportional, (I)ntegral, (D)erivative, and (F)loat
    // P*: how strongly the controller reacts to the current error
    // I*: how strongly the controller reacts to the accumulation of past error (helps eliminate steady-state error)
    // D*: how strongly the controller reacts to the rate of change of the error (helps reduce overshoot and oscillation)
    // F:  provides a baseline output to reach a target speed or position without error
    public final PIDFCoefficients DRIVE_PIDF = new PIDFCoefficients(15.0, 3.0, 0.0, 0.0);
    public final PIDFCoefficients FLYWHEEL_PIDF = new PIDFCoefficients(20.0, 5.0, 1.0, 0.0);
    public final PIDFCoefficients INTAKE_PIDF = new PIDFCoefficients(10.0, 3.0, 0.0, 0.0);
    // END PIDF SETTINGS

    // SYSTEM VARIABLES -- LEAVE BE
    public Gamepad gpDriver;
    public Gamepad gpDriver_previous;
    public Gamepad gpOperator;
    public Gamepad gpOperator_previous;

    // Hardware
    public Mecanum         mecanum;
    public IMU Control_IMU;
    public IMU             Expansion_IMU;
    public ElapsedTime elapsedTime;
    public ElapsedTime     rampTime;
    public DcMotorEx driveFL = null;
    public DcMotorEx       driveFR = null;
    public DcMotorEx       driveRL = null;
    public DcMotorEx       driveRR = null;
    public DcMotorEx       auxFlywheel = null;
    public DcMotorEx       auxIntake = null;
    public Servo auxLoader = null;
    public Servo           auxPusher = null;

    public double rpmToVelocity(double RPM, int countsPerRevolution) {
        return ( RPM / 60 ) * countsPerRevolution;
    }

    public double velocityToRPM(double Velocity, int countsPerRevolution) {
        return ( Velocity / countsPerRevolution ) * 60;
    }

    public void init() {
        // Actuator setup
        this.driveFL = hardwareMap.get(DcMotorEx.class, "FL");
        this.driveFR = hardwareMap.get(DcMotorEx.class, "FR");
        this.driveRL = hardwareMap.get(DcMotorEx.class, "RL");
        this.driveRR = hardwareMap.get(DcMotorEx.class, "RR");

        DcMotorEx[] driveTrainMotors = {driveFL, driveFR, driveRL, driveRR};
        for (DcMotorEx motor : driveTrainMotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DRIVE_PIDF);
        }

        driveFL.setDirection(DcMotorSimple.Direction.REVERSE);
        driveFR.setDirection(DcMotorSimple.Direction.FORWARD);
        driveRL.setDirection(DcMotorSimple.Direction.REVERSE);
        driveRR.setDirection(DcMotorSimple.Direction.FORWARD);

        this.auxIntake = hardwareMap.get(DcMotorEx.class, "Intake");
        auxIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        auxIntake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, INTAKE_PIDF);
        auxIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        auxIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        this.auxFlywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        auxFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        auxFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, FLYWHEEL_PIDF);
        auxFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        auxFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);

        this.auxLoader = hardwareMap.get(Servo.class, "Loader");
        this.auxPusher = hardwareMap.get(Servo.class, "Pusher");

        // IMU Initialization
        this.Control_IMU = hardwareMap.get(IMU.class, "imu");
        Control_IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(CONTROL_LOGO_DIRECTION, CONTROL_USB_DIRECTION)));

        // Drivetrain handlers
        this.mecanum = new Mecanum();

        // Start timers
        this.elapsedTime = new ElapsedTime();
        this.rampTime = new ElapsedTime();
    }

    @Override
    public void loop() {

    }

    // DRIVE HELPERS
    public void startEncoderMotion(
            double speed,
            int flTarget,
            int frTarget,
            int rlTarget,
            int rrTarget
    ) {
        setTargetPositions(flTarget, frTarget, rlTarget, rrTarget);
        setRunToPosition();
        setAllPower(Math.abs(speed));
    }

    public void startEncoderDrive(double speed, double leftInches, double rightInches) {
        int leftTicks  = (int)(leftInches * COUNTS_PER_INCH);
        int rightTicks = (int)(rightInches * COUNTS_PER_INCH);

        startEncoderMotion(
                speed,
                driveFL.getCurrentPosition() + leftTicks,
                driveFR.getCurrentPosition() + rightTicks,
                driveRL.getCurrentPosition() + leftTicks,
                driveRR.getCurrentPosition() + rightTicks
        );
    }
    public double degreesToTurnInches(double degrees) {
        return degrees * Math.PI * TRACK_WIDTH_INCHES / 360.0;
    }
    public void startEncoderTurn(double speed, double inches) {
        int ticks = (int)(inches * COUNTS_PER_INCH);

        startEncoderMotion(
                speed,
                driveFL.getCurrentPosition() + ticks,
                driveFR.getCurrentPosition() - ticks,
                driveRL.getCurrentPosition() + ticks,
                driveRR.getCurrentPosition() - ticks
        );
    }

    public void startEncoderStrafe(double speed, double inches) {
        int ticks = (int)(inches * COUNTS_PER_INCH);

        startEncoderMotion(
                speed,
                driveFL.getCurrentPosition() + ticks,
                driveFR.getCurrentPosition() - ticks,
                driveRL.getCurrentPosition() - ticks,
                driveRR.getCurrentPosition() + ticks
        );
    }

    /** Helper functions */
    public void setTargetPositions(int fl, int fr, int rl, int rr) {
        driveFL.setTargetPosition(fl);
        driveFR.setTargetPosition(fr);
        driveRL.setTargetPosition(rl);
        driveRR.setTargetPosition(rr);
    }

    public void setRunToPosition() {
        for (DcMotorEx m : new DcMotorEx[]{driveFL, driveFR, driveRL, driveRR})
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public boolean encoderMotionBusy() {
        return driveFL.isBusy()
                || driveFR.isBusy()
                || driveRL.isBusy()
                || driveRR.isBusy();
    }
    public boolean allMotorsBusy() {
        return driveFL.isBusy() && driveFR.isBusy() &&
                driveRL.isBusy() && driveRR.isBusy();
    }

    public void setAllPower(double pwr) {
        driveFL.setPower(pwr);
        driveFR.setPower(pwr);
        driveRL.setPower(pwr);
        driveRR.setPower(pwr);
    }

    public void stopAllMotion() {
        setAllPower(0);
        for (DcMotorEx m : new DcMotorEx[]{driveFL, driveFR, driveRL, driveRR})
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // SHOOTER
    // SHOOTER
    public void shooterStateMachine() {
        if(cancelOperatorState) {
            cancelOperatorState = false;
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
    public int state_idle() {
        if (!enterOperatorState) {
            timeStep.reset();
            enterOperatorState = true;
        }

        loaderPosition = LOADER_CLOSED_POSITION;
        pusherPosition = PUSHER_CLOSED_POSITION;

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
            doneFiring = true;
            nextOperatorState = OperatorState.IDLE;
        }

        if (nextOperatorState != operatorState) {
            operatorState = nextOperatorState;
            enterOperatorState = false;
            return 1;
        }
        return 0;
    }

    public void printMotorTelemetry(String name, DcMotorEx motor, DcMotor.RunMode runMode, int counts) {
        PIDFCoefficients pidf = driveFL.getPIDFCoefficients(runMode);
        String pidfString = String.format("P: %.2f I: %.2f D: %.2f F: %.2f", pidf.p, pidf.i, pidf.d, pidf.f);

        telemetry.addLine("========================");
        telemetry.addLine(name);
        telemetry.addData("Power", motor.getPower());
        telemetry.addData("Current (milliamps)", motor.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Velocity", motor.getVelocity());
        telemetry.addData("RPM", this.velocityToRPM(motor.getVelocity(), counts));
        telemetry.addData("Position", motor.getCurrentPosition());
        telemetry.addLine(pidfString);
        telemetry.addLine("========================");
    }
    public void telecom() {
        telemetry.addData("Loader Position", auxLoader.getPosition());
        telemetry.addData("Pusher Position", auxPusher.getPosition());
        this.printMotorTelemetry("Intake", auxIntake, DcMotor.RunMode.RUN_USING_ENCODER, INTAKE_COUNTS_PER_REVOLUTION);
        this.printMotorTelemetry("Flywheel", auxFlywheel, DcMotor.RunMode.RUN_USING_ENCODER, FLYWHEEL_COUNTS_PER_REVOLUTION);
        this.printMotorTelemetry("Front Left", driveFL, DcMotor.RunMode.RUN_USING_ENCODER, DRIVE_COUNTS_PER_REVOLUTION);
        this.printMotorTelemetry("Front Right", driveFR, DcMotor.RunMode.RUN_USING_ENCODER, DRIVE_COUNTS_PER_REVOLUTION);
        this.printMotorTelemetry("Rear Left", driveRL, DcMotor.RunMode.RUN_USING_ENCODER, DRIVE_COUNTS_PER_REVOLUTION);
        this.printMotorTelemetry("Rear Right", driveRR, DcMotor.RunMode.RUN_USING_ENCODER, DRIVE_COUNTS_PER_REVOLUTION);
        // Update data
        telemetry.update();
    }
}
