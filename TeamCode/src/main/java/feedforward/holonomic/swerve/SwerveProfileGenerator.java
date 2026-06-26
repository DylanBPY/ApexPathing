package feedforward.holonomic.swerve;

import core.FollowerConstants;
import feedforward.BaseProfileGenerator;
import geometry.Angle;
import geometry.PathPoint;
import geometry.Vector;
import paths.movements.Path;

public class SwerveProfileGenerator extends BaseProfileGenerator {
    private final FollowerConstants config;

    public SwerveProfileGenerator(FollowerConstants config, Path path) {
        super.path = path;
        this.config = config;
    }

    @Override
    protected double calculateMaxTangentialVelocity(PathPoint point, PathPoint lastPoint, Path path, double maxAngVel, double maxAngAccel) {
        double s = point.getDistanceToEnd_in();
        Vector tangent = point.getFirstDerivative();
        double kappa = point.getSignedCurvature();
        double dKappa = point.getCurvatureDerivative();
        Vector finalTangent = path.getParametricPath().getFirstDerivative(1.0);

        Angle headingAtPoint = path.getInterpolator().getHeadingTarg(s, tangent, finalTangent);
        double fPrime = path.getInterpolator().getHeadingFirstDerivative(s, kappa, finalTangent);
        double fDoublePrime = path.getInterpolator().getHeadingSecondDerivative(s, dKappa, finalTangent);

        double maxPhysicalVel = config.forwardVelocityLimit.getIn();

        double effectiveAngVelLimit = Math.min(config.angularVelocityLimit.getIn(), maxAngVel);
        double effectiveAngAccelLimit = Math.min(config.angularAccelerationLimit.getIn(), maxAngAccel);

        if (Math.abs(fPrime) > 1e-6) {
            double maxVelFromOmega = effectiveAngVelLimit / Math.abs(fPrime);
            maxPhysicalVel = Math.min(maxPhysicalVel, maxVelFromOmega);
        }

        if (Math.abs(fDoublePrime) > 1e-6) {
            double maxVelFromAlpha = Math.sqrt(effectiveAngAccelLimit / Math.abs(fDoublePrime));
            maxPhysicalVel = Math.min(maxPhysicalVel, maxVelFromAlpha);
        }

        double min_v = 0.0;
        double max_v = maxPhysicalVel;
        int iterations = 5;

        for (int i = 0; i < iterations; i++) {
            double mid_v = (min_v + max_v) / 2.0;

            if (evaluatePower(mid_v, kappa, fPrime, fDoublePrime) > 1.0) {
                max_v = mid_v;
            } else {
                min_v = mid_v;
            }
        }

        return Math.min(min_v, maxPhysicalVel);
    }

    private double evaluatePower(double v, double kappa, double fPrime, double fDoublePrime) {
        double transPowerRaw = (v * config.translationalKV) + config.translationalCoeffs.kS;
        double latPowerRaw = Math.abs((v * v * kappa) * config.Kcentripetal);

        double totalTranslationalPower = Math.hypot(transPowerRaw, latPowerRaw);

        double omega = fPrime * v;
        double alpha = fDoublePrime * (v * v);
        double headingKs = (Math.abs(omega) > 1e-6) ? (Math.signum(omega) * config.headingCoeffs.kS) : 0.0;

        double rotPower = Math.abs((omega * config.angularKV) + (alpha * config.angularKA) + headingKs);

        return totalTranslationalPower + rotPower;
    }

    @Override
    protected void evaluatePoint(Path path, PathPoint prev, PathPoint current, double v_prev, double v, double a_t, EvaluationResult outResult) {
        double s = current.getDistanceToEnd_in();
        double kappa = current.getSignedCurvature();
        double dKappa = current.getCurvatureDerivative();
        Vector finalTangent = path.getParametricPath().getFirstDerivative(1.0);

        double fPrime = path.getInterpolator().getHeadingFirstDerivative(s, kappa, finalTangent);
        double fDoublePrime = path.getInterpolator().getHeadingSecondDerivative(s, dKappa, finalTangent);

        double omega = fPrime * v;
        double alpha = (fDoublePrime * (v * v)) + (fPrime * a_t);

        double pForward = (v * config.translationalKV) + (a_t * config.translationalKA) + (Math.signum(v) * config.translationalCoeffs.kS);
        double pLateral = (v * v * kappa) * config.Kcentripetal;

        double headingKs = (Math.abs(omega) > 1e-6) ? (Math.signum(omega) * config.headingCoeffs.kS) : 0.0;
        double pHeading = (omega * config.angularKV) + (alpha * config.angularKA) + headingKs;

        outResult.pForward = Math.abs(pForward);
        outResult.pLateral = Math.abs(pLateral);
        outResult.pHeading = Math.abs(pHeading);

        outResult.totalPower = Math.hypot(outResult.pForward, outResult.pLateral) + outResult.pHeading;
        outResult.maxUtilization = outResult.totalPower;
    }

    @Override
    protected double getMaxTangentialAccel(double currentVel, PathPoint point, Path path, double maxAngAccel) {
        double s = point.getDistanceToEnd_in();
        double kappa = point.getSignedCurvature();
        double dKappa = point.getCurvatureDerivative();
        Vector finalTangent = path.getParametricPath().getFirstDerivative(1.0);

        double fPrime = path.getInterpolator().getHeadingFirstDerivative(s, kappa, finalTangent);
        double fDoublePrime = path.getInterpolator().getHeadingSecondDerivative(s, dKappa, finalTangent);

        double maxPhysicalDecel = config.forwardAccelerationLimit.getIn();
        double effectiveAngAccelLimit = Math.min(config.angularAccelerationLimit.getIn(), maxAngAccel);

        if (Math.abs(fPrime) > 1e-6) {
            double rotationalTorqueBase = Math.signum(fPrime) * fDoublePrime * (currentVel * currentVel);
            double maxDecelFromAlpha = (effectiveAngAccelLimit + rotationalTorqueBase) / Math.abs(fPrime);

            maxPhysicalDecel = Math.min(maxPhysicalDecel, Math.max(0.0, maxDecelFromAlpha));
        }

        return maxPhysicalDecel;
    }

    @Override
    protected double calculateDynamicMaxAccel(double currentVel, PathPoint point, Path path, double maxAngAccel) {
        double s = point.getDistanceToEnd_in();
        double kappa = point.getSignedCurvature();
        double dKappa = point.getCurvatureDerivative();
        Vector finalTangent = path.getParametricPath().getFirstDerivative(1.0);

        double fPrime = path.getInterpolator().getHeadingFirstDerivative(s, kappa, finalTangent);
        double fDoublePrime = path.getInterpolator().getHeadingSecondDerivative(s, dKappa, finalTangent);

        double vFwdConsumed = (currentVel * config.translationalKV) + config.translationalCoeffs.kS;
        double vLatConsumed = Math.abs((currentVel * currentVel * kappa) * config.Kcentripetal);

        double omega = fPrime * currentVel;
        double alphaBase = fDoublePrime * (currentVel * currentVel);
        double headingKs = (Math.abs(omega) > 1e-6) ? config.headingCoeffs.kS : 0.0;
        double rotConsumedBase = Math.abs(omega * config.angularKV) + Math.abs(alphaBase * config.angularKA) + headingKs;

        double maxAvailableTranslation = Math.max(0.0, 1.0 - rotConsumedBase);

        // Solve for dynamicAlpha using the Swerve Quadratic Power Equation
        double c1 = config.translationalKA;
        double c2 = Math.abs(fPrime * config.angularKA);

        double A = (c1 * c1) - (c2 * c2);
        double B = 2.0 * ((vFwdConsumed * c1) + (maxAvailableTranslation * c2));
        double C = (vFwdConsumed * vFwdConsumed) + (vLatConsumed * vLatConsumed) - (maxAvailableTranslation * maxAvailableTranslation);

        double dynamicAlpha = 0.0;

        // If C > 0, the current velocity already exceeds the voltage budget at zero acceleration
        if (C <= 0) {
            if (Math.abs(A) < 1e-6) {
                // Linear collapse if c1 exactly equals c2
                dynamicAlpha = -C / B;
            } else {
                double discriminant = (B * B) - (4.0 * A * C);
                if (discriminant >= 0) {
                    double root1 = (-B + Math.sqrt(discriminant)) / (2.0 * A);
                    double root2 = (-B - Math.sqrt(discriminant)) / (2.0 * A);

                    // Validate roots to filter out extraneous solutions created by squaring
                    if (root1 > 0 && (maxAvailableTranslation - (c2 * root1)) >= 0) {
                        dynamicAlpha = Math.max(dynamicAlpha, root1);
                    }
                    if (root2 > 0 && (maxAvailableTranslation - (c2 * root2)) >= 0) {
                        dynamicAlpha = Math.max(dynamicAlpha, root2);
                    }
                }
            }
        }

        // Cap forward acceleration to avoid exceeding structural angular limits
        double effectiveAngAccelLimit = Math.min(config.angularAccelerationLimit.getIn(), maxAngAccel);

        if (Math.abs(fPrime) > 1e-6) {
            double rotationalTorqueBase = Math.signum(fPrime) * fDoublePrime * (currentVel * currentVel);
            double maxAlpha_at = (effectiveAngAccelLimit - rotationalTorqueBase) / Math.abs(fPrime);

            dynamicAlpha = Math.min(dynamicAlpha, Math.max(0.0, maxAlpha_at));
        }

        return Math.min(config.forwardAccelerationLimit.getIn(), dynamicAlpha);
    }
}