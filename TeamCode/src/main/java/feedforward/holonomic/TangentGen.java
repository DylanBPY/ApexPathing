package feedforward.holonomic;

import feedforward.BaseFeedforwardGen;
import geometry.PathPoint;

public class TangentGen extends BaseFeedforwardGen {

    // Specific constants for this type of drive/interpolation
    private final double kW, kaA, kC, kV, kA, kS;
    private final double w_max, aA_max;

    public TangentGen(double kW, double kaA, double kC, double kV, double kA, double kS, double v_max, double a_max, double w_max, double aA_max) {
        super(v_max, a_max); // Pass the universal kinematics to the base algorithm
        this.kW = kW;
        this.kaA = kaA;
        this.kC = kC;
        this.kV = kV;
        this.kA = kA;
        this.kS = kS;
        this.w_max = w_max;
        this.aA_max = aA_max;
    }

    @Override
    protected double calculateSteadyStateVelocity(PathPoint point) {
        double absK = Math.abs(point.getSignedCurvature());
        double absDK = Math.abs(point.getCurvatureDerivative());

        double A = (absDK * kaA) + (absK * kC);
        double B = (absK * kW) + kV;
        double C = kS - 1.0;

        double maxVelPower;
        if (A < 1e-6) {
            maxVelPower = (1.0 - kS) / kV;
        } else {
            maxVelPower = (-B + Math.sqrt(B * B - 4 * A * C)) / (2.0 * A);
        }

        double maxVelOmega = (absK > 1e-6) ? (w_max / absK) : Double.MAX_VALUE;
        double maxVelAlpha = (absDK > 1e-6) ? Math.sqrt(aA_max / absDK) : Double.MAX_VALUE;

        return Math.min(maxVelPower, Math.min(maxVelOmega, maxVelAlpha));
    }

    @Override
    protected void evaluatePoint(PathPoint prev, PathPoint current, double v_prev, double v, double a_t, EvaluationResult outResult) {
        double k_signed = current.getSignedCurvature();
        double dk_signed = current.getCurvatureDerivative();

        double omega = Math.abs(v * k_signed);
        double alpha = Math.abs((a_t * k_signed) + (v * v * dk_signed));

        outResult.pForward = Math.abs(v * kV + a_t * kA + kS);
        outResult.pLateral = Math.abs(v * v * k_signed * kC);
        outResult.pHeading = Math.abs((omega * kW) + (alpha * kaA));
        outResult.totalPower = outResult.pForward + outResult.pLateral + outResult.pHeading;

        double powerUtil = outResult.totalPower / 1.0;
        double omegaUtil = (w_max > 0) ? (omega / w_max) : 0.0;
        double alphaUtil = (aA_max > 0) ? (alpha / aA_max) : 0.0;

        outResult.maxUtilization = Math.max(powerUtil, Math.max(omegaUtil, alphaUtil));
    }
}