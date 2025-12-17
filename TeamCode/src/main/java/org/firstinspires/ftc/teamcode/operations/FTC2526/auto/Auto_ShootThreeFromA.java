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

import androidx.core.graphics.drawable.IconCompat;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.operations.FTC2526.teleop.FiniteStateControlNew;

@Autonomous(name="Position A: Shoot 3", group="Decode")

public class Auto_ShootThreeFromA extends Auto_Base {
    public ElapsedTime stateStart = new ElapsedTime();
    public DriveSteps driverState = DriveSteps.WAIT;
    public DriveSteps nextDriverState = DriveSteps.WAIT;
    public enum DriveSteps {
        WAIT(0),
        LEAVE(1), // Moves out of starting zone
        TURN_TO_GOAL(1),
        BACKUP(1),
        LAUNCH(1),
        DONE(0); // Turn to face wheel
        public final double startDelay;
        DriveSteps(double startDelay) {
            this.startDelay = startDelay;
        }
    }

    void Auto_shootThreeFromA() {
        numberToShoot = 3;
        flywheelMaxVelocities = new double[]{3500, 4500, 5500};
        flywheelPowerIndex = 3;
    }

    @Override
    public void init() {
        stateStart.reset();
        auxPusher.setPosition(PUSHER_CLOSED_POSITION);
        auxLoader.setPosition(LOADER_CLOSED_POSITION);
        super.init();
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
                driverWAIT();
                break;
            case LEAVE:
                driverLEAVE();
                break;
            case TURN_TO_GOAL:
                driverTURN_TO_GOAL();
                break;
            case BACKUP:
                driverBACKUP();
                break;
            case LAUNCH:
                driverLAUNCH();
                break;
            case DONE:
                driverDONE();
                break;
        }
    }

    // Before starting
    private void driverWAIT() {
        nextDriverState = DriveSteps.LEAVE;
    }

    // Leave Base Area
    private boolean leaveStarted = false;
    private void driverLEAVE() {
        if (!leaveStarted) {
            startEncoderDrive(DRIVE_BASE_SPEED, 88, 88);
            leaveStarted = true;
        }

        if (!encoderMotionBusy()) {
            stopAllMotion();
            nextDriverState = DriveSteps.TURN_TO_GOAL;
        }
    }

    // Turn to Goal
    private boolean turnStarted = false;
    private void driverTURN_TO_GOAL() {
        if (!turnStarted) {
            double turn = degreesToTurnInches(-45);
            startEncoderTurn(TWIST_BASE_SPEED, turn);
            turnStarted = true;
        }

        if (!encoderMotionBusy()) {
            stopAllMotion();
            turnStarted = false;
            nextDriverState = DriveSteps.BACKUP;
        }
    }

    // Back up a bit
    private boolean backupStarted = false;
    private void driverBACKUP() {
        if (!backupStarted) {
            startEncoderDrive(DRIVE_BASE_SPEED, 0, 0);
            backupStarted = true;
        }

        if (!encoderMotionBusy()) {
            stopAllMotion();
            nextDriverState = DriveSteps.LAUNCH;
        }
    }

    // Launch artifacts
    private boolean launchStarted = false;
    private boolean doneFiring = false;
    private void driverLAUNCH() {
        if (!launchStarted) {
            doneFiring = false;
            nextState = FiniteStateControlNew.OperatorState.LOAD_AND_SHOOT;
            launchStarted = true;
        }

        if (doneFiring && operatorState == FiniteStateControlNew.OperatorState.IDLE) {
            launchStarted = false;
            nextDriverState = DriveSteps.DONE;
        }
    }

    // Done!
    private void driverDONE() {
        stopAllMotion();
        telemetry.addLine("DONE WITH OPERATION");
    }


    // SHOOTER
    public void shooterStateMachine() {
        if(cancelState) {
            cancelState = false;
            operatorState = FiniteStateControlNew.OperatorState.IDLE;
            nextState = FiniteStateControlNew.OperatorState.IDLE;
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
    private int state_idle() {
        if (!enterState) {
            timeStep.reset();
            enterState = true;
        }

        loaderPosition = LOADER_CLOSED_POSITION;
        pusherPosition = PUSHER_CLOSED_POSITION;

        // If queued state is not the current running state, switch states for next cycle
        if (nextState != operatorState) {
            operatorState = nextState;
            enterState = false;
            return 1;
        }
        return 0;
    }

    private int state_load_and_shoot() {
        int minVelocity = 1980;
        double time = shootStep.seconds();

        if (!enterState) {
            timeStep.reset();
            shootStep.reset();
            enterState = true;

            ballsShot = 0;

            // Always initialize positions
            loaderPosition = LOADER_CLOSED_POSITION;
            pusherPosition = PUSHER_CLOSED_POSITION;
            flywheelVelocity = this.rpmToVelocity(flywheelMaxVelocities[flywheelPowerIndex], FLYWHEEL_COUNTS_PER_REVOLUTION);
        }

        if (ballsShot < numberToShoot) {
            telemetry.addData("Shoot State", subState.toString());
            telemetry.addData("Shooter Time", time);
            telemetry.addData("State Time", timeStep.seconds());

            switch (subState) {
                case IDLE:
                    if(auxFlywheel.getVelocity() < minVelocity) {
                        System.out.println("!!! Flywheel under minimum velocity, waiting . . .");
                        shootStep.reset();
                        loaderPosition = LOADER_CLOSED_POSITION;
                        pusherPosition = PUSHER_CLOSED_POSITION;
                    } else {
                        System.out.println("!!! Flywheel meets min velocity!");
                        subState = FiniteStateControlNew.ShootSubstate.OPEN_LOADER;
                    }
                    break;

                case OPEN_LOADER:
                    loaderPosition = LOADER_OPEN_POSITION;
                    if (time >= subState.startTime) {
                        subState = FiniteStateControlNew.ShootSubstate.CLOSE_LOADER;
                        shootStep.reset();
                    }
                    break;

                case CLOSE_LOADER:
                    loaderPosition = LOADER_CLOSED_POSITION;
                    if (time >= subState.startTime) {
                        subState = FiniteStateControlNew.ShootSubstate.PUSH;
                        shootStep.reset();
                    }
                    break;

                case PUSH:
                    pusherPosition = PUSHER_OPEN_POSITION;
                    if (time >= subState.startTime) {
                        subState = FiniteStateControlNew.ShootSubstate.RETRACT;
                        shootStep.reset();
                    }
                    break;

                case RETRACT:
                    pusherPosition = PUSHER_CLOSED_POSITION;
                    if (time >= subState.startTime) {
                        subState = FiniteStateControlNew.ShootSubstate.DONE;
                        shootStep.reset();
                    }
                    break;

                case DONE:
                    ballsShot++;
                    subState = FiniteStateControlNew.ShootSubstate.IDLE;
                    break;
            }
        }

        // Finished sequence
        if (ballsShot == numberToShoot) {
            loaderPosition = LOADER_CLOSED_POSITION;
            pusherPosition = PUSHER_CLOSED_POSITION;
            flywheelVelocity = FLYWHEEL_MIN_RPM;
            doneFiring = true;
            nextState = FiniteStateControlNew.OperatorState.IDLE;
        }

        if (nextState != operatorState) {
            operatorState = nextState;
            enterState = false;
            return 1;
        }
        return 0;
    }

}
