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
 * NOTE: NOTICE TANGENCY IS NOT GUARANTEED IN THIS BUILDER. I think this is OK for now as almost any
 * path can be created with B-Splines, and anytime a user wants to add a line, they most likely want
 * to stop before continuing. THIS SHOULD BE CLEARLY COMMUNICATED IN THE DOCS, and MIGHT BE CHANGED IN
 * FUTURE UPDATES.
 * <p>
 * Author: DrPixelCat
 */
public class PathBuilder {
    private final Path path;
    private Pose lastPose;

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
    public PathBuilder bSplineTo(Pose... poses) throws Exception {
        if (poses.length == 0) throw new IllegalArgumentException("A B-Spline must be created with > 1 points!");
        Vector[] vectors = new Vector[poses.length + 1];

        vectors[0] = lastPose.toVec();

        for (int i = 0; i < poses.length; i++) {
            vectors[i + 1] = poses[i].toVec();
        }

        PathSegment curve = new PathSegment(new BSpline(vectors));

        // NOTE: This is a very crude solution to make paths a bit more optimal. The user most
        // likely doesn't want the robot to go to the turning effort of following a path tangent if
        // the end is < 36 inches away. It's faster to just do a linear interpolation.
        // THIS IS a) CONFUSING AND b) NOT REALLY WELL OPTIMIZED. THEREFORE: PROBABLY NEEDS TO BE
        // CHANGED IN A BETTER WAY TO BE MORE OPTIMAL AND OR LESS CONFUSING
        path.addSegment(curve, new HeadingInterpolator(
                curve.getLength_in() >= 36 ?
                        InterpolationStyle.TANGENT_OPTIMAL
                        : InterpolationStyle.SMOOTH_START_TO_END,
                lastPose.getHeadingComponent(),
                poses[poses.length - 1].getHeadingComponent())
        );

        lastPose = poses[poses.length - 1];
        return this;
    }

    /**
     * Overrides the heading interpolation strategy for the most recently added segment.
     * This is designed to be chained immediately after adding a segment (e.g., `.lineTo(...).interpolateWith(...)`).
     *
     * @param interpolator The custom HeadingInterpolator to apply to the preceding segment.
     * @return The current PathBuilder instance for method chaining.
     */
    public PathBuilder interpolateWith(HeadingInterpolator interpolator) {
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

        // NOTE: This is a very crude solution to make paths a bit more optimal. The user most
        // likely doesn't want the robot to go to the turning effort of following a path tangent if
        // the end is < 36 inches away. It's faster to just do a linear interpolation.
        // THIS IS a) CONFUSING AND b) NOT REALLY WELL OPTIMIZED. THEREFORE: PROBABLY NEEDS TO BE
        // CHANGED IN A BETTER WAY TO BE MORE OPTIMAL AND OR LESS CONFUSING
        path.addSegment(line, new HeadingInterpolator(
                line.getLength_in() >= 36 ?
                        InterpolationStyle.TANGENT_OPTIMAL
                        : InterpolationStyle.SMOOTH_START_TO_END,
                lastPose.getHeadingComponent(),
                pose.getHeadingComponent())
        );

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
     * Finalizes the construction process and returns the completed path.
     *
     * @return The fully constructed {@link Path} object ready for execution.
     */
    public Path build() {
        return path;
    }
}