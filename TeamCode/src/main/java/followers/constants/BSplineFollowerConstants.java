package followers.constants;

import controllers.PDSController.PDSCoefficients;
import drivetrains.BaseDrivetrain;
import followers.MovementFollower;
import localizers.BaseLocalizer;

/**
 * B-Spline path follower constants class.
 * Configured via the BSplineTuner OpMode for live dashboard tuning.
 * @author Sohum Arora - 22985 Paraducks
 */
public class BSplineFollowerConstants extends FollowerConstants {

    public PDSCoefficients translationCoeffs = new PDSCoefficients();
    public PDSCoefficients headingCoeffs = new PDSCoefficients();
    public double velocityFF = 0.0;

    // Tolerances
    public double headingTolerance = Math.toRadians(1.0);
    public double distanceTolerance = 0.5;
    public double tTolerance = 0.95;
    public double maxLateralAccel = 0;

    public BSplineFollowerConstants() {}

    @Override
    public MovementFollower build(BaseDrivetrain<?> drivetrain, BaseLocalizer<?> localizer) {
        return new MovementFollower(this, drivetrain, localizer);
    }

    // --- Builder Setters for Constants.java ---

    public BSplineFollowerConstants setTranslationCoeffs(PDSCoefficients translationCoeffs) {
        this.translationCoeffs = translationCoeffs;
        return this;
    }

    public BSplineFollowerConstants setHeadingCoeffs(PDSCoefficients headingCoeffs) {
        this.headingCoeffs = headingCoeffs;
        return this;
    }

    public BSplineFollowerConstants setVelocityFF(double velocityFF) {
        this.velocityFF = velocityFF;
        return this;
    }

    public BSplineFollowerConstants setHeadingTolerance(double headingTolerance) {
        this.headingTolerance = headingTolerance;
        return this;
    }

    public BSplineFollowerConstants setDistanceTolerance(double distanceTolerance) {
        this.distanceTolerance = distanceTolerance;
        return this;
    }

    public BSplineFollowerConstants setTTolerance(double tTolerance) {
        this.tTolerance = tTolerance;
        return this;
    }
    public double getMaxLateralAccel() {
        return maxLateralAccel;
    }
}