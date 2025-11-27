package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotLogger {
    private final DcMotorEx[] motors;
    private final Servo[] servos;
    private final ElapsedTime timer;
    private final Gamepad[] gamepads;

    public RobotLogger(DcMotorEx[] motors, Servo[] servos, ElapsedTime timer, Gamepad[] gamepads) {
        this.motors = motors;
        this.servos = servos;
        this.timer = timer;
        this.gamepads = gamepads;

    }

    public void update() {
    }
}
