package paths;

import paths.heading.InterpolationStyle;
import util.Angle;
import util.Distance;
import util.Pose;

/**
 * This is a test path to demonstrate the capabilities of the new .addPath() method in
 * the PathBuilder
 * We can delete one of these 2 ExamplePathAPI classes later once we decide how we will
 * go about the PathBuilder modifications
 */
public class ExamplePathAPI_V2 {

    public Path testPath() {
        return new PathBuilder(new Pose(0, 0, 0, Distance.Units.INCHES, Angle.Units.RADIANS))

                // Unpack everything inside a single unified routing configuration block!
                .newPath(
                        // 1 & 2. THE B-SPLINE
                        new PathBuilder.Step(
                                PathBuilder.SegmentType.BSPLINE,
                                new Pose(Distance.Units.MILLIMETERS, 600, 0),
                                new Pose(15, 10),
                                new Pose(25, 20, Math.toRadians(90))
                        ),

                        // 2. POINT TURN: Stationary rotation.
                        new PathBuilder.Step(
                                PathBuilder.SegmentType.TURN,
                                new Pose(0, 0, Angle.fromDeg(135).getRad())
                        ),

                        // 3. STRAIGHT LINE: Simple point-to-point translation.
                        new PathBuilder.Step(
                                PathBuilder.SegmentType.LINETO,
                                new Pose(0, 0, Math.toRadians(180))
                        )
                )
                /*
                * Look how easy it becomes! All the paths go in one .newPath() block
                * The one drawback is that you have to call .interpolateSegment() separately outside .newPath()
                * So it isn't quite just one block for all paths, will see if we can work on this later
                * Importantly, this unifies all the .bSplineTo(), .turnTo() and .lineTo() methods using an enum which
                * in my opinion at least is way cleaner :)
                 */

                // 4. IN-LINE OVERRIDE: Overrides the heading strategy of the segment generated directly above it.
                .interpolateSegment(InterpolationStyle.TANGENT_FORWARD)

                // 5. ADVANCED LAMBDA OVERRIDE
                .newPath(
                        new PathBuilder.Step(
                                PathBuilder.SegmentType.BSPLINE,
                                new Pose(10, 10),
                                new Pose(20, 0, 0)
                        )
                )
                .interpolateSegment(s -> new Angle(s * (6 * Math.PI)))

                // 6. FAILSAFE DEMONSTRATION: Missing headings!
                .newPath(
                        new PathBuilder.Step(
                                PathBuilder.SegmentType.LINETO,
                                new Pose(Distance.Units.INCHES, 30, 0)
                        )
                )

                // 7. COMPILE: Locks the path and calculates all the Look-Up Tables.
                .build();
    }
}