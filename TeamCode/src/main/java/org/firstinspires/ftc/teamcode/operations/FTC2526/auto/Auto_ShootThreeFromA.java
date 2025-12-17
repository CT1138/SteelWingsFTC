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

package org.firstinspires.ftc.teamcode.operations.FTC2526.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Forward 24 inches", group="Decode")
public class Auto_ShootThreeFromA extends LinearOpMode {
    private Servo soAux_Stopper = null;

    public final double    LOADER_OPEN_POSITION = 0.3;
    public final double    LOADER_CLOSED_POSITION = 0.45;

    /* Declare OpMode members. */
    private DcMotorEx moDrive_FrontLeft = null;
    private DcMotorEx moDrive_FrontRight = null;
    private DcMotorEx moDrive_RearLeft = null;
    private DcMotorEx moDrive_RearRight = null;

    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 28;     // eg: REV HD Hex Motor
    static final double DRIVE_GEAR_REDUCTION = 12;   // gear ratio
    static final double WHEEL_DIAMETER_INCHES = 2.95275591; // wheel size
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.8;
    static final double TURN_SPEED  = 0.5;
    static final double STRAFE_SPEED = 0.6;

    @Override
    public void runOpMode() {
        soAux_Stopper = hardwareMap.get(Servo.class, "Loader");

        moDrive_FrontLeft = hardwareMap.get(DcMotorEx.class, "FL");
        moDrive_FrontRight = hardwareMap.get(DcMotorEx.class, "FR");
        moDrive_RearLeft = hardwareMap.get(DcMotorEx.class, "RL");
        moDrive_RearRight = hardwareMap.get(DcMotorEx.class, "RR");

        moDrive_FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        moDrive_FrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        moDrive_RearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        moDrive_RearRight.setDirection(DcMotorSimple.Direction.FORWARD);
        soAux_Stopper.setPosition(LOADER_CLOSED_POSITION);
        // Reset encoders
        for (DcMotorEx motor : new DcMotorEx[]{moDrive_FrontLeft, moDrive_FrontRight, moDrive_RearLeft, moDrive_RearRight}) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        telemetry.addLine("Encoders reset. Waiting for start...");
        telemetry.update();
        waitForStart();

        soAux_Stopper.setPosition(LOADER_CLOSED_POSITION);

        // Example autonomous routine
        encoderDrive(DRIVE_SPEED,  48,  48, 5.0);   // Forward

        telemetry.addLine("Path Complete");
        telemetry.update();
        sleep(1000);
    }

    /** Drive forward/backward */
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int flTarget = moDrive_FrontLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        int frTarget = moDrive_FrontRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
        int rlTarget = moDrive_RearLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        int rrTarget = moDrive_RearRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

        setTargetPositions(flTarget, frTarget, rlTarget, rrTarget);
        setRunToPosition();

        runtime.reset();
        setAllPower(Math.abs(speed));

        while (opModeIsActive() && (runtime.seconds() < timeoutS) && allMotorsBusy()) {
            telemetry.addData("Path", "Running to %7d:%7d", flTarget, frTarget);
            telemetry.update();
        }

        stopAllMotion();
    }

    /** Turn in place */
    public void encoderTurn(double speed, double inches, double timeoutS) {
        int flTarget = moDrive_FrontLeft.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        int frTarget = moDrive_FrontRight.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
        int rlTarget = moDrive_RearLeft.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        int rrTarget = moDrive_RearRight.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);

        setTargetPositions(flTarget, frTarget, rlTarget, rrTarget);
        setRunToPosition();

        runtime.reset();
        setAllPower(Math.abs(speed));

        while (opModeIsActive() && (runtime.seconds() < timeoutS) && allMotorsBusy()) {
            telemetry.addData("Turn", "Running to %7d:%7d", flTarget, frTarget);
            telemetry.update();
        }

        stopAllMotion();
    }

    /** Strafe left/right for mecanum drive */
    public void encoderStrafe(double speed, double inches, double timeoutS) {
        int flTarget = moDrive_FrontLeft.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        int frTarget = moDrive_FrontRight.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
        int rlTarget = moDrive_RearLeft.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
        int rrTarget = moDrive_RearRight.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

        setTargetPositions(flTarget, frTarget, rlTarget, rrTarget);
        setRunToPosition();

        runtime.reset();
        setAllPower(Math.abs(speed));

        while (opModeIsActive() && (runtime.seconds() < timeoutS) && allMotorsBusy()) {
            telemetry.addData("Strafe", "Running to %7d:%7d", flTarget, frTarget);
            telemetry.update();
        }

        stopAllMotion();
    }

    /** Helper functions */
    private void setTargetPositions(int fl, int fr, int rl, int rr) {
        moDrive_FrontLeft.setTargetPosition(fl);
        moDrive_FrontRight.setTargetPosition(fr);
        moDrive_RearLeft.setTargetPosition(rl);
        moDrive_RearRight.setTargetPosition(rr);
    }

    private void setRunToPosition() {
        for (DcMotorEx m : new DcMotorEx[]{moDrive_FrontLeft, moDrive_FrontRight, moDrive_RearLeft, moDrive_RearRight})
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private boolean allMotorsBusy() {
        return moDrive_FrontLeft.isBusy() && moDrive_FrontRight.isBusy() &&
                moDrive_RearLeft.isBusy() && moDrive_RearRight.isBusy();
    }

    private void setAllPower(double pwr) {
        moDrive_FrontLeft.setPower(pwr);
        moDrive_FrontRight.setPower(pwr);
        moDrive_RearLeft.setPower(pwr);
        moDrive_RearRight.setPower(pwr);
    }

    private void stopAllMotion() {
        setAllPower(0);
        for (DcMotorEx m : new DcMotorEx[]{moDrive_FrontLeft, moDrive_FrontRight, moDrive_RearLeft, moDrive_RearRight})
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(250);
    }
}
