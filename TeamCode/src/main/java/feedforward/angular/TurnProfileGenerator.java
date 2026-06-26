package feedforward.angular;

import org.firstinspires.ftc.teamcode.apexpathing.Constants;

import feedforward.FeedforwardLut;
import feedforward.MotionParameters;
import paths.movements.Turn;

public class TurnProfileGenerator {
    private double omega_max;
    private double alpha_max;
    private Constants constants = new Constants();

    public TurnProfileGenerator(double omega_max, double alpha_max) {
        this.omega_max = omega_max;
        this.alpha_max = alpha_max;
    }

    public void setConstraints(double omega_max, double alpha_max) {
        this.omega_max = omega_max;
        this.alpha_max = alpha_max;
    }

    public FeedforwardLut generate(Turn turn) {
        // Calculate the absolute angular distance of the turn (Assumes radians)
        double totalAngleRads = Math.abs(turn.getEndPose().getHeading().getShortestAngleTo(turn.getStartPose().getHeading()).getRad());

        // Define structural bounds and target density (~2 degrees per step index)
        double targetRadPerStep = Math.toRadians(2.0);
        int minSteps = 15;
        int maxSteps = 200;

        // Calculate adaptive step size based on total sweep length
        int steps = (int) Math.ceil(totalAngleRads / targetRadPerStep) + 1;
        steps = Math.max(minSteps, Math.min(maxSteps, steps));

        double dTheta = totalAngleRads / (steps - 1);
        MotionParameters[] lut = new MotionParameters[steps];

        // Base Pass
        for (int i = 0; i < steps; i++) {
            lut[i] = new MotionParameters();
            lut[i].setAngularVel(omega_max);
            lut[i].setTangentialVel(0.0); // No forward movement
        }

        // Backward Pass (Deceleration)
        lut[steps - 1].setAngularVel(0.0);
        for (int i = steps - 2; i >= 0; i--) {
            double nextW = lut[i + 1].getAngularVel();

            double maxReachableW = Math.sqrt((nextW * nextW) + (2.0 * alpha_max * dTheta));
            lut[i].setAngularVel(Math.min(lut[i].getAngularVel(), maxReachableW));
        }

        // Forward Pass (Acceleration)
        lut[0].setAngularVel(0.0);
        for (int i = 1; i < steps; i++) {
            double prevW = lut[i - 1].getAngularVel();

            double dynamicAlpha = (
                    1.0 - constants.followerConfig().headingCoeffs.kS - (constants.followerConfig().angularKV * prevW)) / constants.followerConfig().angularKA;

            dynamicAlpha = Math.max(0.0, dynamicAlpha);
            double actualAlpha = Math.min(alpha_max, dynamicAlpha);

            double maxReachableW = Math.sqrt((prevW * prevW) + (2.0 * actualAlpha * dTheta));
            lut[i].setAngularVel(Math.min(lut[i].getAngularVel(), maxReachableW));
        }

        return new FeedforwardLut(lut);
    }
}