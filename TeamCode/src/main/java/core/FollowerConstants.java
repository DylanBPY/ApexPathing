package core;

import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;

import controllers.PDSController.PDSCoefficients;
import geometry.Angle;
import geometry.Dist;

/**
 * Apex Pathing FollowerConstants class
 * Internally assigns the coefficient values determined through tuning directly,
 * thereby eliminating the need to manually tune and set the values in the Constants file!
 *
 * @author Sohum Arora 22985 Paraducks
 */
public class FollowerConstants {

    public enum DrivetrainType {
        COAXIAL_SWERVE,
        DUAL_ACTUATED,
        KIWI,
        MECANUM,
        TANK
    }

    public DrivetrainType drivetrainType = DrivetrainType.MECANUM;

    public PDSCoefficients headingCoeffs = new PDSCoefficients();
    public PDSCoefficients translationalCoeffs = new PDSCoefficients();
    public double velocityFeedbackGain = 0.0;
    public double translationalKV = 0.0, translationalKA = 0.0;
    public double angularKV = 0.0, angularKA = 0.0;
    public double Kcentripetal = 0.0;
    public Dist forwardVelocityLimit = Dist.fromIn(0);
    public Dist forwardAccelerationLimit = Dist.fromIn(0);
    public Dist strafeVelocityLimit = Dist.fromIn(0);
    public Dist strafeAccelerationLimit = Dist.fromIn(0);
    public Dist angularVelocityLimit = Dist.fromIn(0);
    public Dist angularAccelerationLimit = Dist.fromIn(0);
    public Angle headingTolerance = Angle.fromDeg(1.0);
    public Dist distanceTolerance = Dist.fromIn(0.5);

    public FollowerConstants() {
        loadValues();
    }

    private void loadValues() {
        File file = new File("/sdcard/FIRST/FollowerConstants.json");
        if (file.exists()) {
            try {
                BufferedReader reader = new BufferedReader(new FileReader(file));
                StringBuilder sb = new StringBuilder();
                String line;
                while ((line = reader.readLine()) != null) sb.append(line);
                reader.close();

                JSONObject json = new JSONObject(sb.toString());

                String dtString = json.optString("drivetrainType", "MECANUM");
                try {
                    this.drivetrainType = DrivetrainType.valueOf(dtString.toUpperCase());
                } catch (IllegalArgumentException e) {
                    this.drivetrainType = DrivetrainType.MECANUM;
                }

                this.headingCoeffs = new PDSCoefficients(
                        json.optDouble("headingP", 0),
                        json.optDouble("headingD", 0),
                        json.optDouble("headingS", 0), 0);

                double tP = json.optDouble("translationP", 0);
                double tD = json.optDouble("translationD", 0);
                double tS = json.optDouble("translationS", 0);
                this.translationalCoeffs = new PDSCoefficients(tP, tD, tS, 0);

                this.translationalKV = json.optDouble("translationKV", this.translationalKV);
                this.translationalKA = json.optDouble("translationKA", this.translationalKA);
                this.angularKV = json.optDouble("angularKV", this.angularKV);
                this.angularKA = json.optDouble("angularKA", this.angularKA);
                this.Kcentripetal = json.optDouble("KC", this.Kcentripetal);
                this.headingTolerance = Angle.fromDeg(json.optDouble("headingToleranceDeg", 1.0));
                this.distanceTolerance = Dist.fromIn(json.optDouble("distanceToleranceIn", 0.5));

                this.forwardVelocityLimit = Dist.fromIn(json.optDouble(
                        "forwardVelocityLimitInPerSec", 0));
                this.forwardAccelerationLimit = Dist.fromIn(json.optDouble(
                        "forwardVelocityLimitInPerSec2", 0));
                this.strafeVelocityLimit = Dist.fromIn(json.optDouble(
                        "strafeVelocityLimitInPerSec", 0));
                this.strafeAccelerationLimit = Dist.fromIn(json.optDouble(
                        "strafeAccelerationLimitInPerSec2", 0));
                this.angularVelocityLimit = Dist.fromIn(json.optDouble(
                        "angularVelocityLimitRadPerSec", 0));
                this.angularAccelerationLimit = Dist.fromIn(json.optDouble(
                        "angularAccelerationLimitRadPerSec2", 0));
            } catch (Exception ignored) {
                // defaults to 0 values everywhere
            }
        }
    }

    public FollowerConstants getConstants() {
        return this;
    }
}