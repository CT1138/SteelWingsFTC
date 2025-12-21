package org.firstinspires.ftc.teamcode.operations.FTC2526.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@Disabled
@Autonomous(name="Auto - Angles Test", group="TEST")
public class Auto_IMUtest extends Auto_Base{
    @Override
    public void loop() {
        telemetry.addData("Angles", Control_IMU.getRobotYawPitchRollAngles());
        telemetry.addData("Heading", Control_IMU.getRobotAngularVelocity(AngleUnit.DEGREES));
    }
}
