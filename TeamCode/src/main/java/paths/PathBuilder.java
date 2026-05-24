package paths;

import paths.geometry.BSpline;
import paths.geometry.Line;
import paths.heading.HeadingInterpolator;
import paths.heading.InterpolationStyle;
import util.Angle;
import util.Pose;
import util.Vector;

/**
 * A builder class designed to construct a {@link Path} fluently.
 * <p>
 * This class keeps track of the robot's state (its last known pose) to automatically
 * link segments together, ensuring continuous paths without needing to manually
 * pass the start point for every new curve.
 * NOTE: NOTICE C1 (tangent) CONTINUITY IS NOT GUARANTEED IN THIS BUILDER. This is because almost any
 * path can be created with B-Splines, and anytime a user wants to add a line, they most likely want
 * to stop before continuing. THIS SHOULD BE CLEARLY COMMUNICATED IN THE DOCS, and WILL LIKELY
 * BE CHANGED IN NEAR-FUTURE UPDATES TO FORCE C1 CONTINUITY.
 * <p>
 * Author: DrPixelCat
 */
public class PathBuilder {
    private final Path path;
    private Pose lastPose;
    private final InterpolationStyle DEFAULT_INTERPOLATION = InterpolationStyle.SMOOTH_START_TO_END;
    private InterpolationStyle currentStyle = DEFAULT_INTERPOLATION;

    /**
     * Initializes the PathBuilder with the starting location and heading of the robot.
     *
     * @param startPose The initial Pose of the robot at the beginning of the path.
     */
    public PathBuilder(Pose startPose) {
        this.path = new Path();
        this.lastPose = startPose;
    }

    /**
     * Appends a continuous Uniform Cubic B-Spline to the path.
     * The curve automatically begins at the end of the previous segment (or the start pose).
     * By default, this segment will use {@link InterpolationStyle#TANGENT_OPTIMAL} for heading.
     *
     * @param poses A variable number of waypoints to define the B-Spline curve.
     *              The final pose determines the target heading for the default interpolator.
     * @return The current PathBuilder instance for method chaining.
     * @throws IllegalArgumentException If too few points are provided to construct a valid B-Spline.
     */
    public PathBuilder bSplineTo(Pose... poses) {
        if (poses.length == 0)
            throw new IllegalArgumentException("A B-Spline must be created with > 1 points!");

        Vector[] vectors = new Vector[poses.length + 1];
        vectors[0] = lastPose.toVec();

        boolean intermediateWarningSent = false;

        for (int i = 0; i < poses.length; i++) {
            vectors[i + 1] = poses[i].toVec();

            // If an intermediate pose (not the last one) has a defined heading, throw a warning
            if (i < poses.length - 1 && !intermediateWarningSent && Double.isFinite(poses[i].getHeading())) {
                // TODO: Decide what "TBD" is
                path.addWarning("APEX WARNING: Intermediate B-Spline headings are ignored! Only the " +
                        "final pose heading controls the end heading. (Disable this warning in TBD)");
                intermediateWarningSent = true;
            }
        }

        Pose endPose = poses[poses.length - 1];
        PathSegment curve = new PathSegment(new BSpline(vectors));

        path.addSegment(curve, buildSafeInterpolator(lastPose, endPose));

        lastPose = endPose;
        return this;
    }

    /**
     * Overrides the heading interpolation strategy for the most recently added segment.
     * This is designed to be chained immediately after adding a segment (e.g., `.lineTo(...).interpolateWith(...)`).
     *
     * @param interpolator The custom HeadingInterpolator to apply to the preceding segment.
     * @return The current PathBuilder instance for method chaining.
     */
    public PathBuilder interpolatePreviousSegment(HeadingInterpolator interpolator) {
        path.overrideLastInterpolator(interpolator);
        return this;
    }

    /**
     * Appends a straight line segment to the path.
     * The line automatically begins at the end of the previous segment (or the start pose).
     * By default, this segment will use {@link InterpolationStyle#TANGENT_OPTIMAL} for heading.
     *
     * @param pose The target end position and heading for the line.
     * @return The current PathBuilder instance for method chaining.
     */
    public PathBuilder lineTo(Pose pose) {
        PathSegment line = new PathSegment(new Line(lastPose.toVec(), pose.toVec()));

        path.addSegment(line, buildSafeInterpolator(lastPose, pose));

        lastPose = pose;
        return this;
    }

    /**
     * Appends a stationary point-turn to the path.
     * The robot will stay at its current (x, y) coordinate and rotate to the target heading.
     *
     * @param targetHeading The Angle the robot should turn to face.
     * @return The current PathBuilder instance for method chaining.
     */
    public PathBuilder turnTo(Angle targetHeading) {
        path.addTurn(targetHeading);

        // Update the state tracker so the next segment knows our new heading!
        lastPose = new Pose(lastPose.getX(), lastPose.getY(), targetHeading.getRad());

        return this;
    }

    /**
     * Overrides the default (SMOOTH_START_TO_END) heading interpolation strategy for the whole path.
     * For fastest results, use the default for shorter segments and TANGENT_OPTIMAL for longer ones.
     *
     * @param style The style to apply to the whole path
     * @return The current PathBuilder instance for method chaining.
     */
    public PathBuilder setInterpolationStyle(InterpolationStyle style) {
        switch (style) {
            case TANGENT_OPTIMAL:
                currentStyle = InterpolationStyle.TANGENT_OPTIMAL;
                break;
            case TANGENT_FORWARD:
                currentStyle = InterpolationStyle.TANGENT_FORWARD;
                break;
            case SMOOTH_START_TO_END:
                currentStyle = InterpolationStyle.SMOOTH_START_TO_END;
                break;
            default:
                throw new IllegalArgumentException(
                        "You need more parameters for: " + style.name() + "! You can use this style " +
                        "on specific segments with interpolatePreviousSegmentWith(<HeadingInterpolator>)");
        }
        return this;
    }

    /**
     * Finalizes the construction process and returns the completed path.
     *
     * @return The fully constructed {@link Path} object ready for execution.
     */
    public Path build() {
        return path;
    }

    // region Helpers

    /**
     * Safely constructs a HeadingInterpolator, automatically falling back to TANGENT_FORWARD
     * and generating a warning if a user forgot to supply valid headings in their Poses.
     */
    private HeadingInterpolator buildSafeInterpolator(Pose start, Pose end) {
        boolean missingHeading = !Double.isFinite(start.getHeading()) || !Double.isFinite(end.getHeading());

        // If the style requires 2 angles, but we are missing one, fallback and warn
        if (missingHeading && (currentStyle == InterpolationStyle.SMOOTH_START_TO_END || currentStyle == InterpolationStyle.TANGENT_OPTIMAL)) {
            path.addWarning("APEX WARNING: Segment missing start/end heading! Falling back to TANGENT_FORWARD. Use Pose(x, y, heading) to fix this.");
            return new HeadingInterpolator(InterpolationStyle.TANGENT_FORWARD);
        }

        // If they explicitly selected TANGENT_FORWARD, use the 1-argument constructor
        if (currentStyle == InterpolationStyle.TANGENT_FORWARD) {
            return new HeadingInterpolator(InterpolationStyle.TANGENT_FORWARD);
        }

        // Otherwise, we have valid headings and can safely use the 2-angle constructor
        return new HeadingInterpolator(currentStyle, start.getHeadingComponent(), end.getHeadingComponent());
    }

    // endregion
}