package paths;

import core.Follower;
import geometry.Angle;
import geometry.Pose;
import paths.movements.FollowerMovement;
import paths.movements.Path;
import paths.movements.Turn;
import geometry.AngleUnit;
import geometry.DistUnit;
import geometry.GeometryFactory;

public class ExampleAutoPath {
    private final DistUnit distUnit = DistUnit.IN;
    private final AngleUnit angleUnit = AngleUnit.DEG;
    public GeometryFactory factory;

    private static final Pose startPose = Pose.Common.CENTER.get();

    public Path testPath;
    public Turn testTurn;

    public ExampleAutoPath(Follower follower, GeometryFactory.PoseMirror mirror) {
        factory = new GeometryFactory(follower)
                .setDistUnit(distUnit)
                .setAngleUnit(angleUnit)
                .setPoseMirror(mirror);

        buildRoutine();
    }

    public void exampleCallback() {
        // This will run when the follower reaches a specific progression or angle
    }

    /**
     * A comprehensive showcase of the Unified Movement Builder API.
     */
    private void buildRoutine() {

        // 1. THE CORE B-SPLINE
        // Demonstrating standard routing, auto-tightening, and educational warnings
        testPath = factory.path(
                        startPose,
                        factory.pose(15, 0),
                        factory.pose(25, 0, 90),
                        factory.arcPose(25, 25, 10),
                        factory.pose(45, 25, 45)
                )
                .addDistanceCallback(0.5, this::exampleCallback)

                // 3. ANGULAR CALLBACK: Triggers precisely when the robot rotates past the
                // 180-degree mark
                .addAngularCallback(Angle.fromRad(Math.PI), this::exampleCallback)

                // 4. ADVANCED NODE INTERPOLATOR: Uses nodes to specify the heading of the robot
                // at specific s percentages.
                // NOTE: The C2 Natural Cubic Spline requires a minimum of two nodes to compile
                // successfully!
                .addHeadingNode(0.25, Angle.fromDeg(90))
                .addHeadingNode(0.5, Angle.fromDeg(180))
                .addHeadingNode(0.75, Angle.fromDeg(90))

                // 5. COMPILE: Evaluates geometry and solves a complete optimized kinematically
                // constrained feedforward motion profile.
                // Alternatively, use .quickBuild() for pure geometric lookahead tracking without
                // a physics profile.
                .profiledBuild();

        // ---------------------------------------------------------

        // 6. THE TURN BUILDER
        // Seamlessly starts EXACTLY where the last path ended using .getEndPose()
        testTurn = g.turn(testPath.getEndPose())
                // Defines the final heading the robot should rotate to
                .turnTo(Angle.fromRad(Math.PI / 2))

                // Safety validated callback during the spin!
                .addAngularCallback(Angle.fromRad(Math.PI / 3), this::exampleCallback)

                // 7. QUICK BUILD: Locks the turn and finalizes the callback math without running
                // the profile physics.
                // Recommended for stationary turns unless strict dynamic profiling is required
                // via .profiledBuild().
                .quickBuild();
    }

    /**
     * Optional helper to retrieve the full routine for a state machine.
     */
    public FollowerMovement[] getAutoRoutine() {
        return new FollowerMovement[]{testPath, testTurn};
    }
}