package paths.movements;

import geometry.Pose;

/**
 * A marker interface representing any executable navigational action 
 * (e.g., driving a spline path, executing a point turn, or holding a pose).
 */
public interface FollowerMovement {
    /**
     * Gets the expected final pose of the robot after this movement is completed.
     * This is critical for linking sequential builders together!
     * @return The final Pose.
     */
    Pose getEndPose();

    /**
     * Below are methods to track a robot's progress along a given FollowerMovement, enabling the implementation of a robust FSM
     */
    boolean hasStarted();
    boolean hasEnded();

    void setStarted(boolean started);
    void setEnded(boolean ended);
}