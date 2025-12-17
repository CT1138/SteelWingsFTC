package org.firstinspires.ftc.teamcode.operations.FTC2526.auto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.util.Mecanum;

public class Auto_Base {
    //============================================================================================
    // BEGIN VARIABLES
    // EDIT CONTROLS IN loop()
    //============================================================================================
    // SETTINGS
    // CONSTANTS
    public final double    DRIVE_BASE_SPEED = 0.8;
    public final double    TWIST_BASE_SPEED = 0.5;
    public final double    STRAFE_BASE_SPEED = 0.5;

    public final int       DRIVE_COUNTS_PER_REVOLUTION = 28;
    public final int       FLYWHEEL_COUNTS_PER_REVOLUTION = 28;
    public final int       INTAKE_COUNTS_PER_REVOLUTION = 28;

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
    


}
