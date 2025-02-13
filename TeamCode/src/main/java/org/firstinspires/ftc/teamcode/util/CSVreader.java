package org.firstinspires.ftc.teamcode.util;

import android.os.Build;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.csv.CSVFormat;
import org.apache.commons.csv.CSVParser;
import org.apache.commons.csv.CSVPrinter;
import org.apache.commons.csv.CSVRecord;
import org.firstinspires.ftc.teamcode.util.Goal;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileReader;
import java.io.IOException;
import java.io.Reader;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.UUID;

public class CSVreader {
    private static String msPath;
    private static DcMotor[] motors;
    private static Servo[] servos;

    

    public CSVreader(String asPath, DcMotor[] iMotors, Servo[] iServos) {
        msPath = asPath;
        motors = iMotors;
        servos = iServos;
    }

    public Goal[] readCSV() {
        List<Goal> objective = new ArrayList<>();

        try (Reader reader = new BufferedReader(new FileReader(msPath));
             CSVParser csvParser = new CSVParser(reader, CSVFormat.DEFAULT.withHeader())) {

            for (CSVRecord csvRecord : csvParser) {
                // Accessing values by header names
                String msName = csvRecord.get("Name");

                // Positions
                    String msExtendPosition = csvRecord.get("Extend Position");
                    int miExtendPosition = Integer.parseInt(msExtendPosition);

                    String msTwistPosition = csvRecord.get("Twist Position");
                    int miTwistPosition = Integer.parseInt(msTwistPosition);

                    String msElbowPosition = csvRecord.get("Elbow Position");
                    int miElbowPosition = Integer.parseInt(msElbowPosition);

                    boolean mbClawOpen = Boolean.parseBoolean(csvRecord.get("Claw Open"));
                    int miClawPosition = mbClawOpen ? 1 : 0;

                // Powers
                    String msExtendPower = csvRecord.get("Extend Power");
                    double mdExtendPower = Double.parseDouble(msExtendPower);

                    String msTwistPower = csvRecord.get("Twist Power");
                    double mdTwistPower = Double.parseDouble(msTwistPower);

                    String msElbowPower = csvRecord.get("Elbow Power");
                    double mdElbowPower = Double.parseDouble(msElbowPower);

                int[] miPositions = {miExtendPosition, miTwistPosition, miElbowPosition, miClawPosition};
                double[] miPowers = {mdExtendPower, mdTwistPower, mdElbowPower, 1.0};

                Goal goal = new Goal(msName, motors, servos, miPositions, miPowers);
                objective.add(goal);
            }

        } catch (IOException e) {
            e.printStackTrace();
        }
        return objective.toArray(new Goal[0]);
    }

    public void writeTempObjective(int[] mdPositions, double[] mdPowers) throws IOException {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            try (
                BufferedWriter writer = Files.newBufferedWriter(Paths.get("TeamCode/src/main/assets/objective_temporary.csv"));
                CSVPrinter csvPrinter = new CSVPrinter(writer, CSVFormat.DEFAULT);
            ) {
                // "Name", "Extend Position", "Twist Position", "Elbow Position", "Extend Power", "Twist Power", "Elbow Power", "Claw Open"
                csvPrinter.printRecord(
                        "temp",
                        mdPositions[0],
                        mdPositions[1],
                        mdPositions[2],
                        mdPowers[0],
                        mdPowers[1],
                        mdPowers[2],
                        mdPositions[3]
                );

                // Flush the CSVPrinter to ensure the data is written to the file
                csvPrinter.flush();
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
        }
    }

}