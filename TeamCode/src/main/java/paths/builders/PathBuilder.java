package paths.builders;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

import paths.movements.Path;
import paths.callbacks.AngleCallback;
import paths.callbacks.DistanceCallback;
import geometry.BSpline;
import geometry.PathSegment;
import paths.heading.HeadingInterpolator;
import paths.heading.InterpolationStyle;
import geometry.Angle;
import geometry.Vector;
import geometry.ArcPose;
import geometry.Pose;

/**
 * A builder class designed to construct a {@link Path} fluently.
 * <p>
 * This class captures path configurations (waypoints, interpolators, callbacks)
 * in any order and defers geometric compilation until {@link #build()} is called.
 * C2 (tangent and acceleration) continuity is guaranteed in this builder.
 * <p>
 * @author DrPixelCat
 * @author Sohum Arora 22985 Paraducks
 */
public class PathBuilder {
    public Path path;

    // State Tracking
    private final Pose segmentStartPose;
    private Pose expectedEndPose;
    private Pose[] rawPoses = null;

    private InterpolationStyle currentStyle = InterpolationStyle.SMOOTH_START_TO_END;
    private Angle customOffset = null;
    private Function<Double, Angle> customFunction = null;

    // Stores callbacks to be validated and attached during the build process
    private final List<Runnable> buildTasks = new ArrayList<>();

    /**
     * Initializes the PathBuilder with the starting location and heading of the robot.
     *
     * @param startPose The initial Pose of the robot at the beginning of the path.
     */
    public PathBuilder(Pose startPose) {
        this.path = new Path();
        this.segmentStartPose = startPose;
    }

    /**
     * Stores a sequence of control points to define a continuous Uniform Cubic B-Spline.
     * Any {@link ArcPose} provided is dynamically split into two adjacent control points to round sharp corners.
     * <p>
     * Note: Geometric processing is deferred until {@link #build()} is called.
     *
     * @param poses A variable number of waypoints/control points.
     * @return The current PathBuilder instance for method chaining.
     * @throws IllegalArgumentException If endpoints are arc poses or insufficient points are provided.
     * @throws IllegalStateException If control points have already been added to this builder.
     */
    public PathBuilder addControlPoints(Pose... poses) {
        if (this.rawPoses != null) {
            throw new IllegalStateException("Control points have already been added to this builder!");
        }
        if (poses.length < 2) {
            throw new IllegalArgumentException("A B-Spline must be created with > 1 points!");
        }
        if (poses[0] instanceof ArcPose || poses[poses.length - 1] instanceof ArcPose) {
            throw new IllegalArgumentException("Endpoints can't be arcs!");
        }

        this.rawPoses = poses;
        this.expectedEndPose = poses[poses.length - 1];

        return this;
    }

    /**
     * Overrides the default (SMOOTH_START_TO_END) interpolation with a different {@link InterpolationStyle}
     *
     * @param style The interpolation style to apply.
     * @return The current PathBuilder instance for method chaining.
     */
    public PathBuilder interpolateWith(InterpolationStyle style) {
        this.currentStyle = style;
        return this;
    }

    /**
     * Overrides the interpolation style, providing a custom angular offset.
     * Used primarily for {@link InterpolationStyle#TANGENT_CUSTOM}.
     *
     * @param style The interpolation style to apply.
     * @param angleOffset The fixed angle to offset the calculation by.
     * @return The current PathBuilder instance for method chaining.
     */
    public PathBuilder interpolateWith(InterpolationStyle style, Angle angleOffset) {
        this.currentStyle = style;
        this.customOffset = angleOffset;
        return this;
    }

    /**
     * Overrides the default interpolation with a custom function of distance percentage (s).
     *
     * @param function A lambda mapping distance percentage [0.0, 1.0] to a target Angle.
     * @return The current PathBuilder instance for method chaining.
     */
    public PathBuilder interpolateWith(Function<Double, Angle> function) {
        this.currentStyle = InterpolationStyle.CUSTOM_DIST_FUNCTION;
        this.customFunction = function;
        return this;
    }

    /**
     * Attaches an executable callback based on the physical distance percentage.
     *
     * @param s The physical distance percentage [0.0, 1.0].
     * @param action The code to execute.
     * @return The current PathBuilder instance for method chaining.
     */
    public PathBuilder addDistanceCallback(double s, Runnable action) {
        buildTasks.add(() -> path.addCallback(new DistanceCallback(s, action)));
        return this;
    }

    /**
     * Attaches an executable callback based on the robot reaching a target heading.
     *
     * @param angle The Angle at which the callback should trigger.
     * @param action The code to execute.
     * @return The current PathBuilder instance for method chaining.
     */
    public PathBuilder addAngularCallback(Angle angle, Runnable action) {
        buildTasks.add(() -> {
            // We can only definitively pre-calculate boundary sweeps for SMOOTH lerping.
            // Tangent and Custom interpolators sweep dynamically based on curve integration,
            // so we bypass the strict bounds check to prevent falsely crashing the user's build.
            if (currentStyle == InterpolationStyle.SMOOTH_START_TO_END) {
                double startRad = segmentStartPose.getHeading().getRad();
                double endRad = expectedEndPose.getHeading().getRad();

                if (Double.isFinite(startRad) && Double.isFinite(endRad)) {
                    double targetRad = angle.getRad();

                    double totalDiff = getShortestAngularDifference(startRad, endRad);
                    double targetDiff = getShortestAngularDifference(startRad, targetRad);

                    if (Math.abs(totalDiff) < 1e-6) {
                        if (Math.abs(targetDiff) > 1e-6) {
                            throw new IllegalArgumentException("Angular callback out of bounds: The path's target heading is constant.");
                        }
                    } else if ((totalDiff * targetDiff < 0) || (Math.abs(targetDiff) > Math.abs(totalDiff))) {
                        throw new IllegalArgumentException("Angular callback is outside the sweep range of the start and end headings.");
                    }
                }
            }
            path.addCallback(new AngleCallback(angle, action));
        });

        return this;
    }

    /**
     * Compiles all configuration data, calculates new ctrl points from {@link ArcPose}, generates the curve,
     * verifies callback safety, and returns the completed executable Path.
     *
     * @return The fully constructed {@link Path} object ready for execution.
     */
    public Path build() {
        if (rawPoses == null) {
            throw new IllegalStateException("Cannot build path: No control points were added!");
        }

        // 1. Pre-process the points (Expand ArcPoses)
        ArrayList<Pose> processedPoses = new ArrayList<>(rawPoses.length * 2);
        processedPoses.add(rawPoses[0]);

        boolean intermediateWarningSent = false;

        for (int i = 1; i < rawPoses.length - 1; i++) {
            Pose currentPose = rawPoses[i];

            if (!intermediateWarningSent && Double.isFinite(currentPose.getHeading().getRad())) {
                path.addWarning("APEX WARNING: Intermediate B-Spline headings are ignored! Only the " +
                        "final pose heading controls the end heading.");
                intermediateWarningSent = true;
            }

            if (currentPose instanceof ArcPose) {
                ArcPose arcPose = (ArcPose) currentPose;
                double radius = arcPose.getRadius().getIn();

                if (radius < 2.0) {
                    throw new IllegalArgumentException("ArcPose radius must be at least 2.0 inches.");
                }

                Pose prevPose = rawPoses[i - 1];
                Pose nextPose = rawPoses[i + 1];

                Vector vecToLast = prevPose.getPos().minus(arcPose.getPos());
                Vector vecToNext = nextPose.getPos().minus(arcPose.getPos());

                double distToLast = vecToLast.getMag().getIn();
                double distToNext = vecToNext.getMag().getIn();

                if (radius > distToLast) {
                    throw new IllegalArgumentException("ArcPose radius (" + radius + ") exceeds distance to the last control point.");
                } else if (radius > distToNext) {
                    throw new IllegalArgumentException("ArcPose radius (" + radius + ") exceeds distance to the next control point.");
                }

                Vector p1Vec = arcPose.getPos().plus(vecToLast.times(radius / distToLast));
                Vector p2Vec = arcPose.getPos().plus(vecToNext.times(radius / distToNext));

                processedPoses.add(new Pose(p1Vec, arcPose.getHeading()));
                processedPoses.add(currentPose);
                processedPoses.add(new Pose(p2Vec, arcPose.getHeading()));

            } else {
                processedPoses.add(currentPose);
            }
        }

        processedPoses.add(rawPoses[rawPoses.length - 1]);

        // 2. Build the curve using the fully processed points
        Vector[] vectors = new Vector[processedPoses.size() + 1];
        vectors[0] = segmentStartPose.getPos(); // Inherit end of previous segment

        for (int i = 0; i < processedPoses.size(); i++) {
            vectors[i + 1] = processedPoses.get(i).getPos();
        }

        PathSegment curve = new PathSegment(new BSpline(vectors));
        path.setParametricPath(curve);

        // 3. Inject interpolator state
        Angle startH = segmentStartPose.getHeading();
        Angle endH = expectedEndPose.getHeading();

        if (currentStyle == InterpolationStyle.CUSTOM_DIST_FUNCTION) {
            path.setInterpolator(new HeadingInterpolator(customFunction));
        } else {
            boolean missingHeading = !Double.isFinite(startH.getRad()) || !Double.isFinite(endH.getRad());

            if (missingHeading && currentStyle != InterpolationStyle.TANGENT_FORWARD && currentStyle != InterpolationStyle.TANGENT_CUSTOM) {
                path.addWarning("APEX WARNING: Segment missing start/end heading! Falling back to TANGENT_FORWARD. Use pose.of(x, y, heading) to fix.");
                currentStyle = InterpolationStyle.TANGENT_FORWARD;
            }
            path.setInterpolator(new HeadingInterpolator(currentStyle, startH, endH, customOffset));
        }

        // 4. Run deferred tasks (validating boundaries and attaching callbacks)
        for (Runnable task : buildTasks) {
            task.run();
        }

        return path;
    }

    private double getShortestAngularDifference(double from, double to) {
        double diff = (to - from) % (2 * Math.PI);
        if (diff > Math.PI) diff -= 2 * Math.PI;
        else if (diff < -Math.PI) diff += 2 * Math.PI;
        return diff;
    }
}