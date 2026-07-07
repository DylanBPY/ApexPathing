package core;

import android.os.Environment;

import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;

import controllers.PDSController.PDSCoefficients;
import geometry.Angle;
import geometry.Dist;

/**
 * Class to hold constants for the Follower class. These constants are loaded from a JSON file
 * created by the tuners. If the file does not exist or cannot be read, default values will be used.
 *
 * @author Sohum Arora 22985 Paraducks
 * @author Dylan B. 18597 RoboClovers - Delta
 */
public class FollowerConstants {
    public PDSCoefficients headingCoeffs, translationalCoeffs, driveCoeffs, velocityCoeffs;
    public double translationalKV, translationalKA = 0.0;
    public double maxTranslationalAccel = 0.0;
    public Dist velocityLimit = Dist.fromIn(0.0);

    public Angle headingTolerance = Angle.fromDeg(1.0);
    public Dist distanceTolerance = Dist.fromIn(0.5);
    public double tTolerance = 0.95;

    public FollowerConstants() {
        File file = new File(
                Environment.getExternalStorageDirectory().getPath() +
                        "/FIRST/ApexPathing/constants.json"
        );

        if (file.exists()) {
            try {
                BufferedReader reader = new BufferedReader(new FileReader(file));
                StringBuilder sb = new StringBuilder();

                String line;
                while ((line = reader.readLine()) != null) sb.append(line);
                reader.close();

                JSONObject json = new JSONObject(sb.toString());

                this.headingCoeffs = new PDSCoefficients(
                        readDouble(json, "headingP", 0),
                        readDouble(json, "headingD", 0),
                        readDouble(json, "headingS", 0),
                        0
                );
                this.translationalCoeffs = new PDSCoefficients(
                        readDouble(json, "translationalP", 0),
                        readDouble(json, "translationalD", 0),
                        readDouble(json, "translationalS", 0),
                        0
                );
                this.driveCoeffs = new PDSCoefficients(
                        readDouble(json, "driveP", 0),
                        readDouble(json, "driveD", 0),
                        readDouble(json, "driveS", 0),
                        0
                );

                this.translationalKV = readDouble(json, "translationalKV", this.translationalKV);
                this.translationalKA = readDouble(json, "translationalKA", this.translationalKA);
                this.maxTranslationalAccel = readDouble(
                        json, "maxTranslationalAccel", this.maxTranslationalAccel
                );
                this.velocityLimit = Dist.fromIn(
                        readDouble(json, "velocityLimit", this.velocityLimit.getIn())
                );
                this.headingTolerance = Angle.fromDeg(
                        readDouble(json, "headingToleranceRad", this.headingTolerance.getRad())
                );
                this.distanceTolerance = Dist.fromIn(
                        readDouble(json, "distanceToleranceIn", this.distanceTolerance.getIn())
                );
                this.tTolerance = readDouble(json, "tTolerance", this.tTolerance);

            } catch (Exception ignored) {
                // If there's an error reading the file, ignore it and use default values.
            }
        }
    }

    private double readDouble(JSONObject json, String key, double defaultValue) {
        try {
            return json.optDouble(key, defaultValue);
        } catch (Exception e) {
            return defaultValue;
        }
    }

    public JSONObject toJson() {
        JSONObject json = new JSONObject();

        try {
            if (headingCoeffs != null) {
                json.put("headingP", headingCoeffs.kP);
                json.put("headingD", headingCoeffs.kD);
                json.put("headingS", headingCoeffs.kS);
            }

            if (translationalCoeffs != null) {
                json.put("translationalP", translationalCoeffs.kP);
                json.put("translationalD", translationalCoeffs.kD);
                json.put("translationalS", translationalCoeffs.kS);
            }

            if (driveCoeffs != null) {
                json.put("driveP", driveCoeffs.kP);
                json.put("driveD", driveCoeffs.kD);
                json.put("driveS", driveCoeffs.kS);
            }

            if (velocityCoeffs != null) {
                json.put("velocityP", velocityCoeffs.kP);
                json.put("velocityD", velocityCoeffs.kD);
                json.put("velocityS", velocityCoeffs.kS);
            }

            json.put("translationalKV", translationalKV);
            json.put("translationalKA", translationalKA);
            json.put("maxTranslationalAccel", maxTranslationalAccel);
            json.put("velocityLimit", velocityLimit.getIn());
            json.put("headingToleranceRad", headingTolerance.getRad());
            json.put("distanceToleranceIn", distanceTolerance.getIn());
            json.put("tTolerance", tTolerance);
        } catch (Exception ignored) {
            // If JSON serialization fails, return whatever has been populated so far.
        }

        return json;
    }
}