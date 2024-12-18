/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;



/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@Autonomous(name="DriveForward", group="Into-The-Deep")

public class DriveForward extends LinearOpMode {
    private DcMotor Drive_FrontLeft = null;
    private DcMotor Drive_FrontRight = null;
    private DcMotor Drive_RearLeft = null;
    private DcMotor Drive_RearRight = null;
    private DcMotor Arm_Extend = null;
    
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private double[] calculateMecanum(double drive, double strafe, double twist) {
        double frontleft = 0.0;
        double frontright = 0.0;
        double rearleft = 0.0;
        double rearright = 0.0;

        if (!gamepad2.right_bumper) {
            double modifier = 1;
            drive = drive * modifier;
            strafe = strafe * modifier;
            twist = twist * modifier;
        }

        // Calculate Powers
        frontleft = drive + strafe + twist;
        frontright = drive - strafe - twist;
        rearleft = drive - strafe + twist;
        rearright = drive + strafe + twist;

        // Return Powers
        return new double[]{frontleft, frontright, rearleft, rearright};
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        Drive_FrontLeft  = hardwareMap.get(DcMotor.class, "Drive_FrontLeft");
        Drive_FrontRight = hardwareMap.get(DcMotor.class, "Drive_FrontRight");
        Drive_RearLeft   = hardwareMap.get(DcMotor.class, "Drive_RearLeft");
        Drive_RearRight  = hardwareMap.get(DcMotor.class, "Drive_RearRight");
        Arm_Extend = hardwareMap.get(DcMotor.class, "Arm_Extend");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.
            double[] wheelpower = calculateMecanum(0, 0, 0);
    
            if (runtime.time() <= 1 && runtime.time() >= 0) {
                Arm_Extend.setPower(-0.5);
                wheelpower = calculateMecanum(0, 0, 0);
            } else {
                Arm_Extend.setPower(0);
                wheelpower = calculateMecanum(1, 0, 0);
            }

            
            // Setup a variable for each drive wheel to save power level for telemetry
            double frontLeftPower = 0.0;
            double frontRightPower = 0.0;
            double rearLeftPower = 0.0;
            double rearRightPower = 0.0;

            frontLeftPower  = wheelpower[0] ;
            frontRightPower = wheelpower[1] ;
            rearLeftPower   = wheelpower[2] ;
            rearRightPower  = wheelpower[3] ;
    
            // Power Motors
            Drive_FrontLeft.setPower(-frontLeftPower);
            Drive_FrontRight.setPower(frontRightPower);
            Drive_RearLeft.setPower(-rearLeftPower);
            Drive_RearRight.setPower(rearRightPower);

            // Show the elapsed game time and wheel power.
            telemetry.update();
        }
    }
}
