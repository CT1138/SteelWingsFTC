package org.firstinspires.ftc.teamcode.operations.FTC2526.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "Auto - Angle Hold", group = "TEST")
public class Auto_AngleHold extends Auto_Base {

    // ===== SETTINGS =====
    private static final double TARGET_ANGLE = 0.0;
    private static final double ANGLE_TOLERANCE = 2.1; // ±0.1 degrees

    // Proportional gains (START SMALL)
    private static final double kP_YAW   = 0.002;
    private static final double kP_PITCH = 0.003;
    private static final double kP_ROLL  = 0.003;

    // Max correction power
    private static final double MAX_POWER = 0.4;

    @Override
    public void loop() {

        // Read IMU
        YawPitchRollAngles angles = Control_IMU.getRobotYawPitchRollAngles();

        double yaw   = angles.getYaw(AngleUnit.DEGREES);
        double pitch = angles.getPitch(AngleUnit.DEGREES);
        double roll  = angles.getRoll(AngleUnit.DEGREES);

        // Compute errors
        double yawError   = TARGET_ANGLE - yaw;
        double pitchError = TARGET_ANGLE - pitch;
        double rollError  = TARGET_ANGLE - roll;

        // Apply deadband (±0.1°)
        yawError   = applyDeadband(yawError);
        pitchError = applyDeadband(pitchError);
        rollError  = applyDeadband(rollError);

        // Proportional control
        double turn   = yawError   * kP_YAW;
        double drive  = pitchError * kP_PITCH;
        double strafe = rollError  * kP_ROLL;

        // Clamp outputs
        turn   = -Range.clip(turn,   -MAX_POWER, MAX_POWER);
        drive  = Range.clip(drive,  -MAX_POWER, MAX_POWER);
        strafe = Range.clip(strafe, -MAX_POWER, MAX_POWER);
        strafe = 0;
        // Drive mecanum
        double[] wheelPower = mecanum.Calculate(drive, strafe, turn, false);
        //wheelPower = super.tractionControl(wheelPower, DRIVE_SLIP_THRESHOLD);

        double maxVelocity = this.rpmToVelocity(DRIVE_MAX_RPM, DRIVE_COUNTS_PER_REVOLUTION);
        driveFL.setVelocity(wheelPower[0] * maxVelocity);
        driveFR.setVelocity(wheelPower[1] * maxVelocity);
        driveRL.setVelocity(wheelPower[2] * maxVelocity);
        driveRR.setVelocity(wheelPower[3] * maxVelocity);

        // Telemetry
        telemetry.addData("Yaw", yaw);
        telemetry.addData("Pitch", pitch);
        telemetry.addData("Roll", roll);

        telemetry.addData("Yaw Error", yawError);
        telemetry.addData("Pitch Error", pitchError);
        telemetry.addData("Roll Error", rollError);

        telemetry.addData("Drive", drive);
        telemetry.addData("Strafe", strafe);
        telemetry.addData("Turn", turn);

        telemetry.update();
    }

    private double applyDeadband(double error) {
        return Math.abs(error) <= ANGLE_TOLERANCE ? 0.0 : error;
    }
}
