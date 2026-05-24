package paths.geometry;

import util.Vector;

/**
 * Represents a purely mathematical, stateless 2D parametric curve.
 * Evaluated strictly using the parameter 't' in the domain [0.0, 1.0].
 * Author: DrPixelCat
 */
public interface ParametricSegment {

    /**
     * @return The (x, y) coordinate at parameter t
     */
    Vector getPosition(double t);

    /**
     * @return The first derivative (velocity vector) at parameter t
     */
    Vector getFirstDerivative(double t);

    /**
     * @return The second derivative (acceleration vector) at parameter t
     */
    Vector getSecondDerivative(double t);

    /**
     * @return The normalized tangent vector (forward direction) at parameter t.
     */
    default Vector getTangentVector(double t) {
        Vector d1 = getFirstDerivative(t);
        if (d1.getMagnitudeSquared() < 1e-9) {
            return new Vector(0, 0); // Failsafe for singularities
        }
        return d1.normalize();
    }

    /**
     * @return The normalized normal vector (perpendicular to tangent) at parameter t.
     * Used for calculating centripetal force and curvature vectors.
     */
    default Vector getNormalVector(double t) {
        // Rotating the tangent by 90 degrees (PI/2 radians) yields the normal
        return getTangentVector(t).rotated(Math.PI / 2.0);
    }

    /**
     * Calculates the signed curvature at parameter t.
     *
     * @return Curvature (1 / radius). A straight line will naturally return 0.0.
     */
    default double getCurvature(double t) {
        Vector d1 = getFirstDerivative(t);
        Vector d2 = getSecondDerivative(t);

        double magSq = d1.getMagnitudeSquared();
        if (magSq <= 1e-9) {
            return 0.0;
        }

        // κ = (v × a) / |v|^3
        double crossProduct = d1.crossProduct(d2);
        return crossProduct / Math.pow(magSq, 1.5);
    }
}