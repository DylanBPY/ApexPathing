package controllers;

import geometry.Angle;
import geometry.Dist;
import geometry.Vector;

public class MecanumDriveController {
    private final double strafePenaltyRatio;
    public final PDSController pds;
    public final Dist tolerance;

    public MecanumDriveController(Dist maxForwardVelocity, Dist maxStrafeVelocity, PDSController.PDSCoefficients pdsCoefficients, Dist tolerance) {
        this.tolerance = tolerance;
        this.strafePenaltyRatio = maxForwardVelocity.getIn() / maxStrafeVelocity.getIn();
        this.pds = new PDSController(pdsCoefficients);
    }

    /**
     * Helper for basic point-to-point correction.
     */
    public Vector calculatePointToPoint(Vector targetPos, Vector currentPos, Angle currentHeading) {
        Vector fieldError = targetPos.minus(currentPos);
        if (fieldError.getMag().getIn() < 0.01) return Vector.zero();

        double basePower = pds.calculateFromError(fieldError.getMag().getIn());
        Vector rawFieldVector = Vector.fromPolar(Dist.fromIn(basePower), fieldError.getTheta());

        return applyMecanumCorrections(rawFieldVector, currentHeading);
    }

    public Vector applyMecanumCorrections(Vector rawFieldCentricPower, Angle currentHeading) {
        Vector localVector = rawFieldCentricPower.rotate(currentHeading.times(-1.0));

        Vector correctedLocalVector = new Vector(
                Dist.fromIn(localVector.getX().getIn() * strafePenaltyRatio),
                localVector.getY()
        );

        return correctedLocalVector.rotate(currentHeading);
    }
}