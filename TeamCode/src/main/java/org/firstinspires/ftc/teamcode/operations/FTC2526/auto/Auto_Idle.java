package org.firstinspires.ftc.teamcode.operations.FTC2526.auto; // package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.io.IOException;

@Autonomous(name="Auto - Idle", group="Decode")
public class Auto_Idle extends OpMode
{
    // Actuators
    public final double    LOADER_OPEN_POSITION = 0.3;
    public final double    LOADER_CLOSED_POSITION = 0.45;

    private final double[] stopperPositions = {0.5, 0};
    private final double[] intakePowers = {0, 1, 1};
    private DcMotorEx moDrive_FrontLeft = null;
    private DcMotorEx moDrive_FrontRight = null;
    private DcMotorEx moDrive_RearLeft = null;
    private DcMotorEx moDrive_RearRight = null;
    private Servo soAux_Stopper = null;

    // throws IOException as some utility classes I wrote require file operations
    public Auto_Idle() throws IOException {
    }

    // Prep our motors and servos
    @Override
    public void init() {
        // Actuator setup
        moDrive_FrontLeft = hardwareMap.get(DcMotorEx.class, "FL");
        moDrive_FrontRight = hardwareMap.get(DcMotorEx.class, "FR");
        moDrive_RearLeft = hardwareMap.get(DcMotorEx.class, "RL");
        moDrive_RearRight = hardwareMap.get(DcMotorEx.class, "RR");

        soAux_Stopper = hardwareMap.get(Servo.class, "Loader");

        moDrive_FrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        moDrive_FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        moDrive_RearLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        moDrive_RearRight.setDirection(DcMotorSimple.Direction.REVERSE);

        moDrive_FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moDrive_FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moDrive_RearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moDrive_RearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        moDrive_FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moDrive_FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moDrive_RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moDrive_RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        soAux_Stopper.setPosition(LOADER_CLOSED_POSITION);
    }

    public double[] tractionControl(double[] powers, double[] velocities, double slipThreshold) {
        // Calculate the average velocity by adding each value and dividing by the number of values
        double avgVel = (velocities[0] + velocities[1] + velocities[2] + velocities[3]) / 4.0;

        // Loop through each value and calculate how much to reduce its power
        for (int i = 0; i < 4; i++) {
            if (avgVel > 50 && velocities[i] > avgVel * slipThreshold) {

                double slipRatio = velocities[i] / avgVel;
                double reduction = Math.min(0.5, (slipRatio - 1.0) * 0.5);
                System.out.println("Traction Control activated: " + slipRatio);
                telemetry.addLine("Traction Control activated: " + slipRatio);
                powers[i] *= (1.0 - reduction);
            }
        }

        return powers;
    }

    // Reset our timer
    @Override
    public void start() {
       soAux_Stopper.setPosition(stopperPositions[0]);
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
