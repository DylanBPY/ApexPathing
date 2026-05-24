package paths;

import java.util.ArrayList;
import java.util.List;

import paths.heading.HeadingInterpolator;
import util.Angle;

/**
 * Represents a complete, navigable route for the robot to follow.
 * <p>
 * A Path is composed of a sequential series of segments and their associated
 * heading strategies, wrapped together in {@link PathNode}s to guarantee they
 * remain synchronized during execution.
 * <p>
 * Author: DrPixelCat
 */
// IMPROVEMENT NOTE (this is AI note, but I thought I'd leave it here): This class is "stateful" because it
// maintains a `currentIndex`. This means you cannot have two robots run this path
// simultaneously in a simulation, nor can you easily draw the path on a dashboard
// while the robot is driving it (drawing it would advance the index!).
// In the future, consider extracting `currentIndex` into a separate `PathTracker`
// or `Cursor` object so this `Path` class becomes a pure, stateless data container.
public class Path {

    /**
     * A composite wrapper that securely binds a geometric path segment to its
     * corresponding heading interpolation strategy.
     */
    public enum NodeType {
        DRIVE,
        TURN
    }

    public static class PathNode {
        public final NodeType type;

        // Populated if type == DRIVE
        public final PathSegment segment;
        public final HeadingInterpolator interpolator;

        // Populated if type == TURN
        public final Angle targetHeading;

        /**
         * Constructor for a geometric drive segment
         */
        public PathNode(PathSegment segment, HeadingInterpolator interpolator) {
            this.type = NodeType.DRIVE;
            this.segment = segment;
            this.interpolator = interpolator;
            this.targetHeading = null;
        }

        /**
         * Constructor for a stationary turn
         */
        public PathNode(Angle targetHeading) {
            this.type = NodeType.TURN;
            this.targetHeading = targetHeading;
            this.segment = null;
            this.interpolator = null;
        }
    }

    private final List<PathNode> nodes = new ArrayList<>();
    private final List<String> buildWarnings = new ArrayList<>();
    private int currentIndex = 0;

    /**
     * Appends a new segment and its heading strategy to the end of the path.
     *
     * @param segment      The geometric curve to add.
     * @param interpolator The heading strategy for this curve.
     */
    public void addSegment(PathSegment segment, HeadingInterpolator interpolator) {
        nodes.add(new PathNode(segment, interpolator));
    }

    /**
     * Appends a stationary turn to the path
     */
    public void addTurn(Angle targetHeading) {
        nodes.add(new PathNode(targetHeading));
    }

    /**
     * Overwrites the heading interpolator of the most recently added node.
     * Primarily used by the PathBuilder when chaining heading constraints.
     *
     * @param newInterpolator The new heading strategy to apply to the last segment.
     * @throws IllegalStateException if the path is empty.
     */
    public void overrideLastInterpolator(HeadingInterpolator newInterpolator) {
        if (nodes.isEmpty()) throw new IllegalStateException("No nodes to override.");

        int lastIndex = nodes.size() - 1;

        // Retrieve the current last node so we don't lose its geometric segment
        PathNode lastNode = nodes.get(lastIndex);

        // Overwrite the slot in-place with the old segment and the new interpolator
        nodes.set(lastIndex, new PathNode(lastNode.segment, newInterpolator));
    }

    /**
     * Retrieves the node (segment and interpolator pair) that the robot is currently tracking.
     *
     * @return The active {@link PathNode}.
     * @throws IllegalStateException if the path contains no segments.
     */
    public PathNode getCurrentNode() {
        if (nodes.isEmpty()) throw new IllegalStateException("Path is empty!");
        return nodes.get(currentIndex);
    }

    /**
     * Advances the path's internal state to the next segment.
     * If the path is already on the last segment, this method does nothing.
     */
    public void advance() {
        if (!isLastSegment()) {
            currentIndex++;
        }
    }

    /**
     * Adds a per-path warning based on feedback from PathBuilder
     *
     * @param warning The warning string to be displayed on the driver hub
     */
    public void addWarning(String warning) {
        if (!buildWarnings.contains(warning)) { // Prevent spamming the exact same warning twice
            buildWarnings.add(warning);
        }
    }

    /**
     * Gets Path warnings to be displayed to driver
     *
     * @return The list of warnings corresponding to each path segment.
     */
    public List<String> getWarnings() {
        return buildWarnings;
    }

    /**
     * Checks if the robot has reached the final segment of the path.
     *
     * @return True if the current segment is the last one in the list, false otherwise.
     */
    public boolean isLastSegment() {
        return currentIndex >= nodes.size() - 1;
    }

    /**
     * Resets the internal index back to zero so the path can be run again from the beginning.
     * This should be called immediately before a robot begins following this path.
     */
    public void reset() {
        currentIndex = 0;
    }
}