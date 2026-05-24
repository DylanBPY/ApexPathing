package paths.geometry;

import util.Vector;

/**
 * Represents a straight line segment between two points in 2D space.
 * Evaluates linearly from the start point at t = 0 to the end point at t = 1.
 * <p>
 * Author: DrPixelCat
 */
public class Line implements ParametricSegment {

    private final Vector start;
    private final Vector end;
    private final Vector diff; // Cached displacement vector
    private static final Vector zero = new Vector();

    /**
     * Constructs a line segment between two points.
     *
     * @param start The starting point (t = 0.0)
     * @param end   The ending point (t = 1.0)
     */
    public Line(Vector start, Vector end) {
        this.start = start;
        this.end = end;
        this.diff = end.subtract(start);
    }

    /**
     * Calculates the physical (x, y) position on the line at a given percentage.
     *
     * @param t The parameter [0.0, 1.0] representing progress along the line.
     * @return A Vector representing the coordinate location.
     */
    @Override
    public Vector getPosition(double t) {
        if (t < 0.0) {
            t = 0.0;
        } else if (t > 1.0) {
            t = 1.0;
        }
        return start.add(diff.multiply(t));
    }

    /**
     * Calculates the first derivative (velocity vector) of the line.
     * Since velocity is constant along a line, this simply returns the total displacement vector.
     *
     * @param t The parameter [0.0, 1.0].
     * @return A Vector representing the constant parametric velocity.
     */
    @Override
    public Vector getFirstDerivative(double t) {
        return diff;
    }

    /**
     * Calculates the second derivative (acceleration vector) of the line.
     * Since a straight line has no curvature or change in velocity, this always returns zero.
     *
     * @param t The parameter [0.0, 1.0].
     * @return A zero Vector representing no acceleration.
     */
    @Override
    public Vector getSecondDerivative(double t) {
        return zero;
    }

    // Optional Getters

    /**
     * @return The starting coordinate of the line segment.
     */
    public Vector getStart() {
        return start;
    }

    /**
     * @return The ending coordinate of the line segment.
     */
    public Vector getEnd() {
        return end;
    }
}