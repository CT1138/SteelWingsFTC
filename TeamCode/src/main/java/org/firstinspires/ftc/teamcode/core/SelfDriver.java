package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.Mecanum;

public class SelfDriver {
    private double mdHeadingError = 0;
    private IMU moIMU;
    private Mecanum moMecanum = new Mecanum(0.75);

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double mdTargetHeading = 0;
    private double mdDriveSpeed = 0;
    private double mdTurnSpeed = 0;
    private double mdLeftSpeed = 0;
    private double mdRightSpeed = 0;

    private int mdFrontLeftTarget = 0;
    private int mdFrontRightTarget = 0;

    // PARAMETERS:
    static double mdMotorCounts;
    static double mdGearReduction;
    static double mdWheelDiameter;
    static double mdCountsPerInch;

    static final double mdHeadingThreshold = 1.0 ;
    static double mdTurnGain;
    static double mdDriveGain;

    // CLASS DECLARATIONS
    private final DcMotor[] moLeftMotors;
    private final DcMotor[] moRightMotors;
    private final DcMotor[] moMotors;

    /**
     * Constructor for SelfDriver
     * @param aoIMU IMU to use for heading
     * @param asLogoDirection Direction of the Control hub's logo relative to the robot
     * @param asUSBDirection Direction of the Control hub's USB port relative to the robot
     * @param aoLeftMotors all motors on the left side of the drivetrain
     * @param aoRightMotors all motors on the right side of the drivetrain
     * @param adMotorCounts The number of counts per motor revolution. Check the documentation for your motor to get this value
     * @param adGearReduction The gear reduction between your motor and your wheel. For gearing UP, use a gear ratio less than 1.0, and for gearing DOWN, use a gear ratio greater than 1.0
     * @param adWheelDiameter The Diameter of your wheels in INCHES.
     * @param adTurnGain The gain factor applied to heading error to obtain turning power. 0.02 is a good value. Larger values are more responsive but less stable
     * @param adDriveGain The gain factor applied to heading error to obtain drive power. 0.06 is a good value. Larger values are more responsive but less stable
     */
    public SelfDriver(IMU aoIMU, String asLogoDirection, String asUSBDirection, DcMotor[] aoLeftMotors, DcMotor[] aoRightMotors, double adMotorCounts, double adGearReduction, double adWheelDiameter, double adTurnGain, double adDriveGain) {
        mdTurnGain = adTurnGain;
        mdDriveGain = adDriveGain;

        // Set Motors
            moLeftMotors = aoLeftMotors;
            moRightMotors = aoRightMotors;

            moMotors = new DcMotor[moLeftMotors.length + moRightMotors.length];
            System.arraycopy(moLeftMotors, 0, moMotors, 0, moLeftMotors.length);
            System.arraycopy(moRightMotors, 0, moMotors, moLeftMotors.length, moRightMotors.length);

        // Set Settings
            mdMotorCounts = adMotorCounts;
            mdGearReduction = adGearReduction;
            mdWheelDiameter = adWheelDiameter;
            mdCountsPerInch = (mdMotorCounts * mdGearReduction) / (mdWheelDiameter * 3.1415);

        // Build IMU
            moIMU = aoIMU;
            RevHubOrientationOnRobot moIMUConfig = configureIMU(asLogoDirection, asUSBDirection);
            moIMU.initialize(new IMU.Parameters(moIMUConfig));
            moIMU.resetYaw();
    }

    // **********  HIGH Level driving functions.  ********************

    /**
     *  Drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the OpMode running.
     *
     * @param adMaxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param adDistance   Distance (in inches) to move from current position.  Negative distance means move backward.
     * @param adHeading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from the current robotHeading.
     */
    public void driveStraight(double adMaxDriveSpeed,
                              double adDistance,
                              double adHeading) {

    // Ensure that the OpMode is still active

        int miMoveCounts = (int)(adDistance * mdCountsPerInch);

        for(DcMotor moMotor : moLeftMotors)  {
            // Determine new target position, and pass to motor controller
                mdFrontLeftTarget = moMotor.getCurrentPosition() + miMoveCounts;
            // Set Target FIRST, then turn on RUN_TO_POSITION
                moMotor.setTargetPosition(mdFrontLeftTarget);
                moMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        for(DcMotor moMotor : moRightMotors) {
            // Determine new target position, and pass to motor controller
                mdFrontRightTarget = moMotor.getCurrentPosition() + miMoveCounts;
            // Set Target FIRST, then turn on RUN_TO_POSITION
                moMotor.setTargetPosition(mdFrontRightTarget);
                moMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        // Set the required driving speed  (must be positive for RUN_TO_POSITION)
        // Start driving straight, and then enter the control loop
        adMaxDriveSpeed = Math.abs(adMaxDriveSpeed);
        moveRobot(adMaxDriveSpeed, 0);

        // keep looping while we are still active, and BOTH motors are running.
        while (moMecanum.areMotorsBusy(moMotors))
        {
            // Determine required steering to keep on heading
            mdTurnSpeed = getSteeringCorrection(adHeading, mdDriveGain);

            // if driving in reverse, the motor correction also needs to be reversed
            if (adDistance < 0)
                mdTurnSpeed *= -1.0;

            // Apply the turning correction to the current driving speed.
            moveRobot(mdDriveSpeed, mdTurnSpeed);
        }

        // Stop all motion & Turn off RUN_TO_POSITION
        moveRobot(0, 0);
        for(DcMotor moMotor : moMotors) moMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    /**
     *  Spin on the central axis to point in a new direction.
     *  <p>
     *  Move will stop if either of these conditions occur:
     *  <p>
     *  1) Move gets to the heading (angle)
     *  <p>
     *  2) Driver stops the OpMode running.
     *
     * @param adMaxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param adHeading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double adMaxTurnSpeed, double adHeading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(adHeading, mdDriveGain);

        // keep looping while we are still active, and not on heading.
        while (Math.abs(mdHeadingError) > mdHeadingThreshold) {

            // Determine required steering to keep on heading
            mdTurnSpeed = getSteeringCorrection(adHeading, mdTurnGain);

            // Clip the speed to the maximum permitted value.
            mdTurnSpeed = Range.clip(mdTurnSpeed, -adMaxTurnSpeed, adMaxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, mdTurnSpeed);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    /**
     *  Obtain & hold a heading for a finite amount of time
     *  <p>
     *  Move will stop once the requested time has elapsed
     *  <p>
     *  This function is useful for giving the robot a moment to stabilize its heading between movements.
     *
     * @param adMaxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
     * @param adHeading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param adHoldTime   Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double adMaxTurnSpeed, double adHeading, double adHoldTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (holdTimer.time() < adHoldTime) {
            // Determine required steering to keep on heading
            mdTurnSpeed = getSteeringCorrection(adHeading, mdTurnGain);

            // Clip the speed to the maximum permitted value.
            mdTurnSpeed = Range.clip(mdTurnSpeed, -adMaxTurnSpeed, adMaxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, mdTurnSpeed);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    // **********  LOW Level driving functions.  ********************

    /**
     * Use a Proportional Controller to determine how much steering correction is required.
     *
     * @param adDesiredHeading        The desired absolute heading (relative to last heading reset)
     * @param adProportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double adDesiredHeading, double adProportionalGain) {
        mdTargetHeading = adDesiredHeading;  // Save for telemetry

        // Determine the heading current error
        mdHeadingError = mdTargetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (mdHeadingError > 180)  mdHeadingError -= 360;
        while (mdHeadingError <= -180) mdHeadingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(mdHeadingError * adProportionalGain, -1, 1);
    }

    /**
     * Take separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param adDrive forward motor speed
     * @param adTurn  clockwise turning motor speed.
     */
    public void moveRobot(double adDrive, double adTurn) {
        mdDriveSpeed = adDrive;     // save this value as a class member so it can be used by telemetry.
        mdTurnSpeed = adTurn;      // save this value as a class member so it can be used by telemetry.

        mdLeftSpeed = adDrive - adTurn;
        mdRightSpeed = adDrive + adTurn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double mdMax = Math.max(Math.abs(mdLeftSpeed), Math.abs(mdRightSpeed));
        if (mdMax > 1.0)
        {
            mdLeftSpeed /= mdMax;
            mdRightSpeed /= mdMax;
        }

        double mdLeftBias = 1.0;  // Reduce to <1.0 if left motors are stronger
        double mdRightBias = 1.0; // Reduce to <1.0 if right motors are stronger

        double mdLeftSpeed = this.mdLeftSpeed * mdLeftBias;
        double mdRightSpeed = this.mdRightSpeed * mdRightBias;

        for(DcMotor moMotor : moLeftMotors) {
            moMotor.setPower(mdLeftSpeed);
        }
        for(DcMotor moMotor : moRightMotors) {
            moMotor.setPower(mdRightSpeed);
        }
    }

    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    public double getHeading() {
        YawPitchRollAngles moOrientation = moIMU.getRobotYawPitchRollAngles();
        return moOrientation.getYaw(AngleUnit.DEGREES);
    }

    private RevHubOrientationOnRobot configureIMU(String asLogoDirection, String asUSBDirection) {
        RevHubOrientationOnRobot.LogoFacingDirection moLogoDirection;
        RevHubOrientationOnRobot.UsbFacingDirection moUSBDirection;

        // Match LogoFacingDirection using switch
        switch (asLogoDirection.toUpperCase()) {
            case "FORWARD":
                moLogoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
                break;
            case "BACKWARD":
                moLogoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
                break;
            case "UP":
                moLogoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
                break;
            case "DOWN":
                moLogoDirection = RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
                break;
            case "LEFT":
                moLogoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
                break;
            case "RIGHT":
                moLogoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
                break;
            default:
                throw new IllegalArgumentException("Invalid LogoFacingDirection: " + asLogoDirection);
        }

        // Match UsbFacingDirection using switch
        switch (asUSBDirection.toUpperCase()) {
            case "UP":
                moUSBDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
                break;
            case "DOWN":
                moUSBDirection = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;
                break;
            case "LEFT":
                moUSBDirection = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
                break;
            case "RIGHT":
                moUSBDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
                break;
            default:
                throw new IllegalArgumentException("Invalid UsbFacingDirection: " + asUSBDirection);
        }

        // Return the configured RevHubOrientationOnRobot object
        return new RevHubOrientationOnRobot(moLogoDirection, moUSBDirection);
    }

}