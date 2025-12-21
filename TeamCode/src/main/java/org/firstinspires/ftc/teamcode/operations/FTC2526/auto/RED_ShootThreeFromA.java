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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.operations.FTC2526.teleop.FiniteStateControlNew;

@Autonomous(name="RED A: Shoot 3", group="RED")

public class RED_ShootThreeFromA extends Auto_CCMoveAndShootBase {
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
                driverLEAVE(DriveSteps.TURN_TO_GOAL, leaveSpeed, distanceA);
                break;
            case TURN_TO_GOAL:
                // Positive turns clockwise, negative turns Counterclockwise
                driverTURN_TO_GOAL(DriveSteps.BACKUP, 0.4, turnAngleA);
                break;
            case BACKUP:
                driverBACKUP(DriveSteps.LAUNCH, 0.5, -backupDistance);
                break;
            case LAUNCH:
                driverLAUNCH(DriveSteps.DONE);
                break;
            case DONE:
                driverDONE(DriveSteps.DONE);
                break;
        }
    }
}
