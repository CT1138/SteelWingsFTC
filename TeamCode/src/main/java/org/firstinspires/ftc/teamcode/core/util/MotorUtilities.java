package org.firstinspires.ftc.teamcode.core.util;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.Robot;

public class MotorUtilities {
    private final Robot moRobot;

    /**
     * Constructor
     * @param aoRobot Robot Object from org.firstinspires.ftc.teamcode.core.Robot
     */
    public MotorUtilities(Robot aoRobot) {
        moRobot = aoRobot;
    }

    // =======
    // METHODS
    // =======
    /**
     * Setup multiple motors with the same parameters based off of an array of motor handles
     * @param aoMotors array of motor handles
     * @param asDirection direction for the motors to run
     * @param asMode mode for the motors to run in
     * @param asZeroPowerBehavior zero power behaviors
     */
    public void batchSetup(DcMotor[] aoMotors, String asDirection, String asMode, String asZeroPowerBehavior) {
        for (DcMotor aoMotor : aoMotors) {
            setup(aoMotor, asDirection, asMode, asZeroPowerBehavior);
        }
    }

    /**
     * Setup multiple motors with the same parameters based off of an array of motor names
     * @param asMotorNames array of motor names
     * @param asDirection direction for motors to run
     * @param asMode mode for the motors to run in
     * @param asZeroPowerBehavior zero power behaviors
     */
    public void batchSetup(String[] asMotorNames, String asDirection, String asMode, String asZeroPowerBehavior) {
        for (String asMotorName : asMotorNames) {
            setup(asMotorName, asDirection, asMode, asZeroPowerBehavior);
        }
    }

    /**
     * Setup the motor
     * @param aoMotor handle to the current motor
     * @param asDirection direction for the motor to run
     * @param asMode mode for the motor to begin with
     * @param asZeroPowerBehavior zero power behavior
     */
    public void setup(DcMotor aoMotor, String asDirection, String asMode, String asZeroPowerBehavior) {
        setReversed(aoMotor, asDirection);
        setMode(aoMotor, asMode);
        setZeroPowerBehavior(aoMotor, asZeroPowerBehavior);
    }

    /**
     * Setup the motor
     * @param asMotor string name of the motor
     * @param asDirection direction for the motor to run
     * @param asMode mode for the motor to begin with
     * @param asZeroPowerBehavior zero power behavior
     */
    public void setup(String asMotor,String asDirection, String asMode, String asZeroPowerBehavior) {
        setReversed(moRobot.motor(asMotor), asDirection);
        setMode(moRobot.motor(asMotor), asMode);
        setZeroPowerBehavior(moRobot.motor(asMotor), asZeroPowerBehavior);
    }

    /**
     * Stops all motors given
     * @param asMotors Array of motor names
     */
    public void freeze(String[] asMotors) {
        for (String asMotor : asMotors) {
            moRobot.motor(asMotor).setPower(0);
            moRobot.motor(asMotor).setTargetPosition(moRobot.motor(asMotor).getCurrentPosition());
        }
    }

    // =======
    // Setters
    // =======

    /**
     * Set the motor direction
     * @param aoMotor Motor Handle
     * @param asDirection Direction as a string (not case sensitive)
     */
    public void setReversed(DcMotor aoMotor, String asDirection) {
        switch (asDirection.toUpperCase()) {
            case "REVERSE":
                aoMotor.setDirection(DcMotor.Direction.REVERSE);
                break;
            case "FORWARD":
                aoMotor.setDirection(DcMotor.Direction.FORWARD);
                break;
            default:
                break;
        }
    }

    /**
     * Set the run mode
     * @param aoMotor Motor Handle
     * @param asMode Mode as a string (not case or space sensitive)
     */
    public void setMode(DcMotor aoMotor, String asMode) {
        switch (asMode.toUpperCase().replace(" ", "_")) {
            case "RESET_ENCODERS":
                aoMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;
            case "RUN_TO_POSITION":
                aoMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            case "RUN_USING_ENCODER":
                aoMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;
            case "RUN_WITHOUT_ENCODER":
                aoMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                break;
            default:
                break;
        }
    }

    /**
     * Set the zero power behavior
     * @param aoMotor Motor Handle
     * @param asZeroPowerBehavior Zero Power Behavior as a string (not case sensitive)
     */
    public void setZeroPowerBehavior(DcMotor aoMotor, String asZeroPowerBehavior) {
        switch(asZeroPowerBehavior.toUpperCase()) {
            case "BRAKE":
                aoMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                break;
            case "FLOAT":
                aoMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                break;
            default:
                break;
        }
    }

}
