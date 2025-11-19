package org.firstinspires.ftc.teamcode.util;

import android.os.Environment;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class RobotLogger {

    private DcMotorEx[] motors;
    private Servo[] servos;
    private ElapsedTime timer;

    private File logFile;
    private Environment environment;


    /**
     * @param motors array of DcMotorEx motors to log
     * @param servos array of Servo objects to log
     * @param timer  ElapsedTime instance for timestamps
     * @param fileName name of the log file (e.g. "run1.json")
     */
    public RobotLogger(DcMotorEx[] motors, Servo[] servos, ElapsedTime timer, String fileName) {
        this.motors = motors;
        this.servos = servos;
        this.timer = timer;
        environment = new Environment();

        // Save to /sdcard/FIRST/logs/
        File dir = new File("/FIRST/logs/");
        if (!dir.exists()) dir.mkdirs();

        this.logFile = new File(dir, fileName);

        try {
            if (!this.logFile.exists()) {
                this.logFile.createNewFile();
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Call this every loop() to append a JSON telemetry entry.
     */
    public void update() {
        try (FileWriter writer = new FileWriter(logFile, true)) {

            JSONObject root = new JSONObject();
            root.put("timestamp", timer.milliseconds());

            // Log motors
            JSONArray motorArray = new JSONArray();
            for (DcMotorEx motor : motors) {
                JSONObject m = new JSONObject();
                m.put("port", motor.getPortNumber());
                m.put("velocity", motor.getVelocity());
                m.put("position", motor.getCurrentPosition());
                m.put("power", motor.getPower());
                motorArray.put(m);
            }
            root.put("motors", motorArray);

            // Log servos
            JSONArray servoArray = new JSONArray();
            for (Servo servo : servos) {
                JSONObject s = new JSONObject();
                s.put("port", servo.getPortNumber());
                s.put("position", servo.getPosition());
                servoArray.put(s);
            }
            root.put("servos", servoArray);

            // Write entry + newline
            writer.write(root.toString());
            writer.write("\n");

        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
