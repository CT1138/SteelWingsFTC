package org.firstinspires.ftc.teamcode.FTC2425.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.core.SelfDriver;

@Autonomous(name="Park from Left", group="Robot")
@Disabled
public class Auto_ParkFromLeft extends LinearOpMode {

    /* Declare OpMode members. */
    SelfDriver moDriver;
    private DcMotor[] moMotors;

    private void sequence() {
        double mdDriveSpeed = 0.5;
        double mdTurnSpeed = 0.5;

        //safe distance
        moDriver.driveStraight(mdDriveSpeed, 2.0, 0.0);

        //face observation zone
        moDriver.turnToHeading(mdTurnSpeed, -90.0);

        //approach
        moDriver.driveStraight(mdDriveSpeed, 5.0, 0.0);
        moDriver.turnToHeading(mdTurnSpeed, -90);

        moDriver.driveStraight(mdDriveSpeed, 5.0, 0.0);
        moDriver.turnToHeading(mdTurnSpeed, -90);

        moDriver.driveStraight(mdDriveSpeed, 8.0, 0.0);
        moDriver.turnToHeading(mdTurnSpeed, -90);
        moDriver.turnToHeading(mdTurnSpeed, 0);
    }

    // LESS IMPORTANT STUFF

    private void setup() {
        // Motor definitions:
        DcMotor moDrive_FrontLeft = hardwareMap.get(DcMotor.class, "Drive_FrontLeft");
        DcMotor moDrive_FrontRight = hardwareMap.get(DcMotor.class, "Drive_FrontRight");
        DcMotor moDrive_RearLeft = hardwareMap.get(DcMotor.class, "Drive_RearLeft");
        DcMotor moDrive_RearRight = hardwareMap.get(DcMotor.class, "Drive_RearRight");

            // Build Arrays
        DcMotor[] moLeftMotors = new DcMotor[]{moDrive_FrontLeft, moDrive_RearLeft};
        DcMotor[] moRightMotors = new DcMotor[]{moDrive_FrontRight, moDrive_RearRight};

            moMotors = new DcMotor[moLeftMotors.length + moRightMotors.length];
            System.arraycopy(moLeftMotors, 0, moMotors, 0, moLeftMotors.length);
            System.arraycopy(moRightMotors, 0, moMotors, moLeftMotors.length, moRightMotors.length);

            moDrive_FrontLeft.setDirection(DcMotor.Direction.FORWARD); moDrive_RearLeft.setDirection(DcMotor.Direction.FORWARD);
            moDrive_FrontRight.setDirection(DcMotor.Direction.REVERSE); moDrive_RearRight.setDirection(DcMotor.Direction.REVERSE);

            // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
            for(DcMotor moMotor : moMotors)
                moMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            for(DcMotor moMotor : moMotors)
                moMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Control/Expansion Hub IMU
        IMU moIMU = hardwareMap.get(IMU.class, "imu");

        // Define Self Driver
        moDriver = new SelfDriver(
                moIMU,
                "FORWARD",
                "UP",
                moLeftMotors,
                moRightMotors,
                28,
                80,
                2.95,
                0.02,
                0.06
        );
    }

    @Override
    public void runOpMode() {
        // Run Setup Function
        setup();

        // Wait for the game to start (Display Gyro value while waiting)
        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", moDriver.getHeading());
            telemetry.update();
        }

        // Set the encoders for closed loop speed control, and reset the heading.
        for(DcMotor moMotor : moMotors) moMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sequence();

        telemetry.addData("Path", "Complete");

        telemetry.update();
        sleep(1000);  // Pause to display last telemetry message.
    }
}