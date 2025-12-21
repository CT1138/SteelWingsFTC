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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name="BLUE A: Shoot 3", group="BLUE")
public class Auto_CCMoveAndShootBase extends Auto_CCBase {
    public double leaveSpeed = 0.6;
    public double turnAngleA = 50;
    public double turnAngleB = 135;

    public double leaveDistance = 30;
    public double backupDistance = 5;
    public double distanceA = 80;
    public double distanceB = 44;
    public double distanceC = 52;
    public ElapsedTime stateStart = new ElapsedTime();
    public DriveSteps driverState = DriveSteps.WAIT;
    public DriveSteps nextDriverState = DriveSteps.WAIT;
    public enum DriveSteps {
        FORCE(0),
        WAIT(0),
        LEAVE(0), // Moves out of starting zone
        TURN_TO_GOAL(0),
        BACKUP(0),
        LAUNCH(0),
        DONE(0); // Turn to face wheel
        public final double startDelay;
        DriveSteps(double startDelay) {
            this.startDelay = startDelay;
        }
    }

    @Override
    public void init() {
        stateStart.reset();
        super.init();
        numberToShoot = 3;
        flywheelMaxVelocities = new double[]{3200, 4500, 5500};
        flywheelPowerIndex = 1;
        auxPusher.setPosition(super.PUSHER_CLOSED_POSITION);
        auxLoader.setPosition(super.LOADER_CLOSED_POSITION);
    }

    @Override
    public void loop() {
        this.driverStateMachine();
        this.shooterStateMachine();

        super.loop();
    }
    // DRIVETRAIN
    // State Handler
    public void driverStateMachine() {
        if(nextDriverState != driverState) {
            driverState = nextDriverState;
            stateStart.reset();
        }
        if(stateStart.seconds() < driverState.startDelay) return;
        switch (driverState) {
            case WAIT:
                driverWAIT(DriveSteps.LEAVE);
                break;
            case LEAVE:
                // 80% Speed, forward 88 Inches
                driverLEAVE(DriveSteps.TURN_TO_GOAL, 0.8, 88);
                break;
            case TURN_TO_GOAL:
                // Positive turns clockwise, negative turns Counterclockwise
                driverTURN_TO_GOAL(DriveSteps.BACKUP, 0.4, -45);
                break;
            case BACKUP:
                driverBACKUP(DriveSteps.LAUNCH,0.4, 0);
                break;
            case LAUNCH:
                driverLAUNCH(DriveSteps.DONE);
                break;
            case DONE:
                driverDONE(DriveSteps.DONE);
                break;
        }
    }

    // Before starting
    public void driverWAIT(DriveSteps nextState) {
        nextDriverState = nextState;
    }

    // Leave Base Area
    public boolean leaveStarted = false;
    public void driverLEAVE(DriveSteps nextState, double speed, double inches) {
        if (!leaveStarted) {
            //startEncoderDrive(DRIVE_BASE_SPEED * speed, inches, inches);
            startEncoderDriveIMU(DRIVE_BASE_SPEED * speed, inches);
            leaveStarted = true;
        }

        if (!encoderMotionBusy()) {
            stopAllMotion();
            nextDriverState = nextState;
        }
    }

    // Turn to Goal
    public boolean turnStarted = false;
    public void driverTURN_TO_GOAL(DriveSteps nextState, double adSpeed, double adTurn) {
        if (!turnStarted) {
            double turn = degreesToTurnInches(adTurn);
            startEncoderTurn(TWIST_BASE_SPEED * adSpeed, turn);
            turnStarted = true;
        }

        if (!encoderMotionBusy()) {
            stopAllMotion();
            turnStarted = false;
            nextDriverState = nextState;
        }
    }

    // Back up a bit
    public boolean backupStarted = false;
    public void driverBACKUP(DriveSteps nextState, double speed, double inches) {
        if (!backupStarted) {
            startEncoderDrive(DRIVE_BASE_SPEED * speed, inches, inches);
            backupStarted = true;
        }

        if (!encoderMotionBusy()) {
            stopAllMotion();
            nextDriverState = nextState;
        }
    }

    // Launch artifacts
    public boolean launchStarted = false;
    public void driverLAUNCH(DriveSteps nextState) {
        if (!launchStarted) {
            doneFiring = false;
            nextOperatorState = OperatorState.LOAD_AND_SHOOT;
            launchStarted = true;
        }

        if (doneFiring && operatorState == OperatorState.IDLE) {
            launchStarted = false;
            nextDriverState = nextState;
        }
    }

    // Done!
    public void driverDONE(DriveSteps nextState) {
        stopAllMotion();
        telemetry.addLine("DONE WITH OPERATION");
    }

    public void forceState(DriveSteps state) {
        doneFiring = false;
        launchStarted = false;
        backupStarted = false;
        leaveStarted = false;
        nextDriverState = state;
    }

}
