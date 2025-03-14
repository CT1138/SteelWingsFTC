package org.firstinspires.ftc.teamcode.ClubFair25.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ClubFair25.FairObjectives;
import org.firstinspires.ftc.teamcode.core.Objective;
import org.firstinspires.ftc.teamcode.core.Task;
import org.firstinspires.ftc.teamcode.core.util.CSVManager;

@TeleOp(name="Club Fair Demo", group="Into-The-Deep")
public class ClubFairDemo extends OpMode
{
    // ===============
    // BEGIN VARIABLES
    // ===============
    // Robot Hardware Setup
    private DcMotor Lift;
    private DcMotor Elbow;
    private DcMotor Wrist;
    private Servo Claw;
    private TouchSensor Touch = null;
    // Runtimes
    private final ElapsedTime Runtime = new ElapsedTime();
    private final ElapsedTime ObjectiveRuntime = new ElapsedTime();
    // Objectives
    private CSVManager csv = new CSVManager();
    private Objective moCurrentObjective;
    private FairObjectives moFairObjectives;

    // Robot Hardware Data
    private DcMotor[] Motors;
    private Servo[] Servos;
    // ===============
    // END VARIABLES
    // ===============

    // ===============
    // BEGIN METHODS
    // ===============
    @Override
    public void init() {
        // Single execution on INIT

        // Define Robot
        Touch = hardwareMap.get(TouchSensor.class, "TouchSensor");

        Lift = hardwareMap.get(DcMotor.class, "Lift");
        Elbow = hardwareMap.get(DcMotor.class, "Elbow");
        Wrist = hardwareMap.get(DcMotor.class, "Wrist");

        Claw = hardwareMap.get(Servo.class, "Claw");

        // Set Direction for Lift different than the rest
        Lift.setDirection(DcMotor.Direction.FORWARD);
        Elbow.setDirection(DcMotor.Direction.REVERSE);
        Wrist.setDirection(DcMotor.Direction.REVERSE);


        // Build motor array
        Motors = new DcMotor[]{ Lift, Elbow, Wrist };
        Servos = new Servo[]{ Claw };

        moFairObjectives = new FairObjectives(Motors, Servos);
    }
    @Override
    public void start() {
        ObjectiveRuntime.reset();
        Runtime.reset();
        Claw.setPosition(1);
    }
    private boolean canMoveOn() {
        for (DcMotor motor : Motors) if (motor.isBusy() && motor.getCurrentPosition() != motor.getTargetPosition()) return false;
        return (Runtime.seconds() > 1);
    }
    private void liftarm() {
        boolean mbFloor = Touch.isPressed();

        // Objective Declarations
        Objective foHappi = moFairObjectives.foHappi(1);
        Objective foHang = moFairObjectives.foHang(1);
        Objective foZero = moFairObjectives.foZero(1);
        Objective foWave = moFairObjectives.foWave(0.75);
        Objective foGrabFromFloor = moFairObjectives.foGrabFromFloor(1);
        Objective foGrabFromLowerRung = moFairObjectives.foGrabFromLowerRung(1);

        // ========================
        // BEGIN OBJECTIVE CONTROLS
        // ========================

        // STOP EVERYTHING
        // Control = Left Bumper + Right Bumper + Back
        if (gamepad2.left_bumper && gamepad2.right_bumper && gamepad2.back) {
            if (moCurrentObjective != null) moCurrentObjective.stop();

            for (DcMotor motor : Motors) {
                motor.setPower(0);
                motor.setTargetPosition(motor.getCurrentPosition());
            }
        }

        // Happi Happi
        // Control = X
        if (canMoveOn() && gamepad2.x) {
            Runtime.reset();
            Task[] csvTasks = csv.toTasks("/storage/emulated/0/Download/Objective_Happi.csv", true);
            Objective csvObjective = new Objective(Motors, Servos, csvTasks);
            csvObjective.run(ObjectiveRuntime, mbFloor);
        }

        // Wave at the viewer!
        // Control = A
        if (canMoveOn() && gamepad2.a) {
            Runtime.reset();
            Task[] csvTasks = csv.toTasks("/storage/emulated/0/Download/Objective_Wave.csv", true);
            Objective csvObjective = new Objective(Motors, Servos, csvTasks);
            csvObjective.run(ObjectiveRuntime, mbFloor);
        }

        // Grab Sample From Lower Rung
        // Control = B + Dpad_Left
        if (canMoveOn() && gamepad2.b && gamepad2.dpad_left) {
            Runtime.reset();
            Task[] csvTasks = csv.toTasks("/storage/emulated/0/Download/Objective_GrabSampleFromLowerRung.csv", true);
            Objective csvObjective = new Objective(Motors, Servos, csvTasks);
            csvObjective.run(ObjectiveRuntime, mbFloor);
        }

        // Grab Sample From Floor
        // Control = B + Dpad_Down
        if (canMoveOn() && gamepad2.b && gamepad2.dpad_down) {
            Runtime.reset();
            Task[] csvTasks = csv.toTasks("/storage/emulated/0/Download/Objective_GrabSampleFromFloor.csv", true);
            Objective csvObjective = new Objective(Motors, Servos, csvTasks);
            csvObjective.run(ObjectiveRuntime, mbFloor);
        }

        // 2nd level hang
        // Control = B + Dpad_Up
        if (canMoveOn() && gamepad2.b && gamepad2.dpad_up) {
            Runtime.reset();
            Task[] csvTasks = csv.toTasks("/storage/emulated/0/Download/Objective_LevelTwoHang.csv", true);
            Objective csvObjective = new Objective(Motors, Servos, csvTasks);
            csvObjective.run(ObjectiveRuntime, mbFloor);
        }

        // Return Motors to Zero
        // Control = Back or Y
        if (canMoveOn() && (gamepad2.back || gamepad2.y)) {
            for(DcMotor motor : Motors) {
                motor.setTargetPosition(0);
                motor.setPower(1);
            }
        }

        // ======================
        // END OBJECTIVE CONTROLS
        // ======================
    }
    private void changeMotorMode(DcMotor.RunMode mode) {
        for (DcMotor motor : Motors) {
            motor.setMode(mode);
        }
    }

    @Override
    public void loop() {
        liftarm();

        telemetry.update();
    }

    public void stop() {
        for (DcMotor motor : Motors) {
            motor.setPower(0);
            motor.setTargetPosition(0);
        }
    }

    // ===============
    // END METHODS
    // ===============
}
