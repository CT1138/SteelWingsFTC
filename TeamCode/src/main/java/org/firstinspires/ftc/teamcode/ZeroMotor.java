package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Zero Positions", group="Utility")
public class ZeroMotor extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor Drive_FrontLeft = null;
    private DcMotor Drive_FrontRight = null;
    private DcMotor Drive_RearLeft = null;
    private DcMotor Drive_RearRight = null;
    private DcMotor Arm_Extend = null;
    private DcMotor Arm_PhaseTwo = null;

    private DcMotor Arm_Twist = null;

    private Servo ServoClaw = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        Drive_FrontLeft  = hardwareMap.get(DcMotor.class, "Drive_FrontLeft");
        Drive_FrontRight = hardwareMap.get(DcMotor.class, "Drive_FrontRight");
        Drive_RearLeft   = hardwareMap.get(DcMotor.class, "Drive_RearLeft");
        Drive_RearRight  = hardwareMap.get(DcMotor.class, "Drive_RearRight");
        Arm_Extend = hardwareMap.get(DcMotor.class, "Arm_Extend");
        Arm_PhaseTwo = hardwareMap.get(DcMotor.class, "Arm_PhaseTwo");
        Arm_Twist = hardwareMap.get(DcMotor.class, "Arm_Twist");
        ServoClaw = hardwareMap.get(Servo.class, "Servo_Claw");
        // Wait for the game to start (driver presses START)
        waitForStart();

        DcMotor[] motors = {Drive_FrontLeft, Drive_FrontRight, Drive_RearLeft, Drive_RearRight, Arm_Extend, Arm_PhaseTwo, Arm_Twist};
        for (DcMotor m : motors) {
            m.setTargetPosition(0);
            m.setPower(1);
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(1000);
            telemetry.addLine("Motor " + m.getDeviceName() + " has been zeroed.");
            telemetry.update();
            while(m.isBusy() && m.getCurrentPosition() == m.getTargetPosition()) {
                Thread.yield();
            }
        }

        telemetry.clearAll();
        telemetry.addLine("All motors have been zeroed.");
        telemetry.update();
        sleep(1000);
        stop();
    }
}
