package paths.builders;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

import paths.movements.Path;
import paths.callbacks.Callback;
import geometry.BSpline;
import geometry.PathSegment;
import paths.heading.HeadingInterpolator;
import paths.heading.InterpolationStyle;
import geometry.Angle;
import geometry.Vector;
import geometry.ArcPose;
import geometry.Pose;

public class PathBuilder {
    private final Path path = new Path();
    private final Pose[] rawPoses;
    private final Pose startPose;
    private final Pose expectedEndPose;

    private InterpolationStyle currentStyle = InterpolationStyle.SMOOTH_START_TO_END;
    private Angle customOffset = null;
    private Function<Double, Angle> customFunction = null;

    private final List<Runnable> buildTasks = new ArrayList<>();

    protected PathBuilder(Pose... poses) {
        if (poses.length < 2) {
            throw new IllegalArgumentException("A B-Spline must be created with > 1 points!");
        }
        if (poses[0] instanceof ArcPose || poses[poses.length - 1] instanceof ArcPose) {
            throw new IllegalArgumentException("Endpoints can't be arcs!");
        }
        this.rawPoses = poses;
        this.startPose = poses[0];
        this.expectedEndPose = poses[poses.length - 1];
    }

    public PathBuilder interpolateWith(InterpolationStyle style) {
        this.currentStyle = style;
        return this;
    }

    public PathBuilder interpolateWith(InterpolationStyle style, Angle angleOffset) {
        this.currentStyle = style;
        this.customOffset = angleOffset;
        return this;
    }

    public PathBuilder interpolateWith(Function<Double, Angle> function) {
        this.currentStyle = InterpolationStyle.CUSTOM_DIST_FUNCTION;
        this.customFunction = function;
        return this;
    }

    public PathBuilder addDistanceCallback(double s, Runnable action) {
        buildTasks.add(() -> path.addCallback(new Callback(s, action)));
        return this;
    }

    public PathBuilder addAngularCallback(Angle angle, Runnable action) {
        buildTasks.add(() -> {
            if (currentStyle == InterpolationStyle.SMOOTH_START_TO_END) {
                Angle startRad = rawPoses[0].getHeading();
                Angle endRad = expectedEndPose.getHeading();

                if (Double.isFinite(startRad.getRad()) && Double.isFinite(endRad.getRad())) {
                    double totalDiff = startRad.getShortestAngularDifferenceTo(endRad).getRad();
                    double targetDiff = startRad.getShortestAngularDifferenceTo(angle).getRad();

                    if (Math.abs(totalDiff) < 1e-6) {
                        if (Math.abs(targetDiff) > 1e-6) {
                            throw new IllegalArgumentException("Angular callback out of bounds: The path's target heading is constant.");
                        }
                    } else if ((totalDiff * targetDiff < 0) || (Math.abs(targetDiff) > Math.abs(totalDiff))) {
                        throw new IllegalArgumentException("Angular callback is outside the sweep range of the start and end headings.");
                    }
                }
            }
            path.addCallback(new Callback(angle, action));
        });
        return this;
    }

    public Path build() {
        ArrayList<Pose> processedPoses = new ArrayList<>(rawPoses.length * 2);
        processedPoses.add(rawPoses[0]);
        boolean intermediateWarningSent = false;

        for (int i = 1; i < rawPoses.length - 1; i++) {
            Pose currentPose = rawPoses[i];
            if (!intermediateWarningSent && Double.isFinite(currentPose.getHeading().getRad())) {
                path.addWarning("APEX WARNING: Intermediate B-Spline headings are currently ignored!");
                intermediateWarningSent = true;
            }

            if (currentPose instanceof ArcPose) {
                ArcPose arcPose = (ArcPose) currentPose;
                double radius = arcPose.getRadius().getIn();
                if (radius < 2.0) throw new IllegalArgumentException("ArcPose radius must be at least 2.0 inches.");

                Pose prevPose = rawPoses[i - 1];
                Pose nextPose = rawPoses[i + 1];
                Vector vecToLast = prevPose.getPos().minus(arcPose.getPos());
                Vector vecToNext = nextPose.getPos().minus(arcPose.getPos());

                double distToLast = vecToLast.getMag().getIn();
                double distToNext = vecToNext.getMag().getIn();

                if (radius > distToLast || radius > distToNext) {
                    throw new IllegalArgumentException("ArcPose radius exceeds adjacent distance bounds.");
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

        Vector[] vectors = new Vector[processedPoses.size()];
        for (int i = 0; i < processedPoses.size(); i++) {
            vectors[i] = processedPoses.get(i).getPos();
        }

        path.setParametricPath(new PathSegment(new BSpline(vectors)));

        Angle startH = startPose.getHeading();
        Angle endH = expectedEndPose.getHeading();

        boolean missingParams =
                (currentStyle == InterpolationStyle.CONSTANT_START_HEADING && !Double.isFinite(startH.getRad())) ||
                        (currentStyle == InterpolationStyle.CONSTANT_END_HEADING && !Double.isFinite(endH.getRad())) ||
                        (currentStyle == InterpolationStyle.TANGENT_CUSTOM && (customOffset == null || !Double.isFinite(customOffset.getRad()))) ||
                        (currentStyle == InterpolationStyle.SMOOTH_START_TO_END && (!Double.isFinite(startH.getRad()) || !Double.isFinite(endH.getRad()))) ||
                        (currentStyle == InterpolationStyle.CUSTOM_DIST_FUNCTION && customFunction == null);

        if (missingParams) {
            path.addWarning("APEX WARNING: " + currentStyle.name() + " missing params! Falling back to TANGENT_FORWARD.");
            currentStyle = InterpolationStyle.TANGENT_FORWARD;
        }

        path.setInterpolator(new HeadingInterpolator(currentStyle, startH, endH, customOffset));

        for (Runnable task : buildTasks) {
            task.run();
        }

        return path;
    }
}