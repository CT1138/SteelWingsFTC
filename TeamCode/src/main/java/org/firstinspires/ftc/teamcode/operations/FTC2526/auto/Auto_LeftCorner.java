package org.firstinspires.ftc.teamcode.operations.FTC2526.teleop; // package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.core.util.Mecanum;

import java.io.IOException;
import java.util.TreeMap;

@Autonomous(name="Auto - Left Corner", group="Decode")
public class Auto_LeftCorner extends OpMode
{
    // Actuators
    private DcMotorEx moDrive_FrontLeft = null;
    private DcMotorEx moDrive_FrontRight = null;
    private DcMotorEx moDrive_RearLeft = null;
    private DcMotorEx moDrive_RearRight = null;

    // throws IOException as some utility classes I wrote require file operations
    public Auto_LeftCorner() throws IOException {
    }

    // Prep our motors and servos
    @Override
    public void init() {
        // Actuator setup
        moDrive_FrontLeft = hardwareMap.get(DcMotorEx.class, "FL");
        moDrive_FrontRight = hardwareMap.get(DcMotorEx.class, "FR");
        moDrive_RearLeft = hardwareMap.get(DcMotorEx.class, "RL");
        moDrive_RearRight = hardwareMap.get(DcMotorEx.class, "RR");

        moDrive_FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        moDrive_FrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        moDrive_RearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        moDrive_RearRight.setDirection(DcMotorSimple.Direction.FORWARD);

        moDrive_FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moDrive_FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moDrive_RearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moDrive_RearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        moDrive_FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moDrive_FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moDrive_RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moDrive_RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    // Reset our timer
    @Override
    public void start() {
        double speed = 0.1;

        // Strafe Left
        moDrive_FrontLeft.setPower(speed);
        moDrive_FrontRight.setPower(-speed);
        moDrive_RearLeft.setPower(-speed);
        moDrive_RearRight.setPower(speed);
    }

    // Method to store telemetry data
    public void telecom() {
        telemetry.addLine("===================================");
        telemetry.addLine("DriveTrain");
        telemetry.addLine("===================================");
        telemetry.addData("Front Left Power", moDrive_FrontLeft.getPower());
        telemetry.addData("Front Right Power", moDrive_FrontRight.getPower());
        telemetry.addData("Rear Left Power", moDrive_RearLeft.getPower());
        telemetry.addData("Rear Right Power", moDrive_RearRight.getPower());

        telemetry.addData("Rear Left Velocity", moDrive_RearLeft.getVelocity());
        telemetry.addData("Rear Right Velocity", moDrive_RearRight.getVelocity());
        telemetry.addData("Front Left Velocity", moDrive_FrontLeft.getVelocity());
        telemetry.addData("Front Right Velocity", moDrive_FrontRight.getVelocity());

        telemetry.addData("Front Left Position", moDrive_FrontLeft.getCurrentPosition());
        telemetry.addData("Front Right Position", moDrive_FrontRight.getCurrentPosition());
        telemetry.addData("Rear Left Position", moDrive_RearLeft.getCurrentPosition());
        telemetry.addData("Rear Right Position", moDrive_RearRight.getCurrentPosition());
        telemetry.addLine("===================================");
        telemetry.addLine("Auxiliary");
        telemetry.addLine("===================================");

        // Update data
        telemetry.update();
    }


    // I like to keep loop() fairly minimal and only reference other methods when possible
    @Override
    public void loop() {


        telecom();
    }

}
